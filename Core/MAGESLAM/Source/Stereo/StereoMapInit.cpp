// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "StereoMapInit.h"

#include "Utils/cv.h"
#include "Data/Data.h"
#include "Map/InitializationData.h"
#include "Tracking/Keyframebuilder.h"
#include "Tracking/Reprojection.h"
#include "Tracking/FeatureMatcher.h"
#include "Tracking/MapInitialization.h"
#include "Device/CameraCalibration.h"
#include "Bow/BaseBow.h"
#include "Image/ImagePreprocessor.h"
#include "MageUtil.h"
#include "Utils/Logging.h"
#include "Utils/MageConversions.h"
#include <boost/format.hpp>
#include <memory>

#include <opencv2/core/core.hpp>

namespace mage
{
    namespace
    {
        int CreateFilteredDescriptorMask(const int octave, const cv::Rect& crop, const std::shared_ptr<mage::AnalyzedImage> &frame, std::vector<bool>& useDescriptor)
        {
            int trueCount = 0;
            for (unsigned int i = 0; i < frame->GetDescriptorsCount(); i++)
            {
                const cv::KeyPoint& kpt = frame->GetKeyPoint(i);
                if ((octave < 0 || kpt.octave == octave) && IsWithinArea(kpt.pt, crop))
                {
                    useDescriptor[i] = true;
                    trueCount += 1;
                }
                else
                {
                    useDescriptor[i] = false;
                }
            }
            return trueCount;
        }

        void InitializeWithFrames(
            gsl::span<const cv::DMatch> matches,
            const std::shared_ptr<const AnalyzedImage>& referenceFrame, //query
            const std::shared_ptr<const AnalyzedImage>& currentFrame,   //train
            InitializationData& initializationData,
            const Pose& explicitPose,
            float pixelMaxEpipolarDistance,
            float minAcceptanceDistanceRatio,
            thread_memory memory)
        {
            SCOPE_TIMER(MapInitialization::InitializeWithFrames);

            std::vector<std::pair<cv::DMatch, cv::Point3f>> initial3DPoints;

            std::vector<cv::Point2f> frame1_points;
            std::vector<cv::Point2f> frame2_points;
            MapInitialization::CollectMatchPoints(referenceFrame, currentFrame, matches, frame1_points, frame2_points);

            Pose referencePose{};
            {
                MapInitialization::TriangulatePoints(referenceFrame, currentFrame, referencePose, explicitPose, matches, frame1_points, frame2_points, pixelMaxEpipolarDistance, minAcceptanceDistanceRatio, initial3DPoints);
            }

            // create the mappoint proxies to link the keyframe builders to
            initializationData.Clear();

            std::transform(initial3DPoints.cbegin(), initial3DPoints.cend(), std::back_inserter(initializationData.MapPoints),
                [](const auto& point3D)
            {
                // MapInitialization map points are special and get treated as though they have been adjusted once already (which is technically true)
                return MapPointTrackingProxy::CreateNew(point3D.second, 1);
            });

            const std::shared_ptr<KeyframeBuilder> refKeyframeBuilder = initializationData.AddKeyframeBuilder(referenceFrame, referencePose);
            const std::shared_ptr<KeyframeBuilder> currentKeyframeBuilder = initializationData.AddKeyframeBuilder(currentFrame, explicitPose);

            for (size_t iterator = 0; iterator < initial3DPoints.size(); iterator++)
            {
                refKeyframeBuilder->AddAssociation(initializationData.MapPoints[iterator], initial3DPoints[iterator].first.queryIdx);
                currentKeyframeBuilder->AddAssociation(initializationData.MapPoints[iterator], initial3DPoints[iterator].first.trainIdx);
            }
        }

    }

    StereoMapInit::StereoMapInit(const StereoMapInitializationSettings& settings, mira::determinator& determinator)
        : m_settings{ settings }, m_determinator{ determinator }
    {}

    //assumes undistorted input images with null distortion calibrations
    boost::optional<InitializationData> StereoMapInit::Initialize(const std::shared_ptr<AnalyzedImage>& frame0, const std::shared_ptr<AnalyzedImage>& frame1, const cv::Matx44f& frame0ToFrame1, thread_memory memory)
    {
        // get crop region
        cv::Rect frame0CropRect{ ToCVRect(CalculateOverlapCropSourceInTarget(ToMageMat(frame0ToFrame1), frame0->GetUndistortedCalibration().CreateCameraModel(), frame1->GetUndistortedCalibration().CreateCameraModel(), m_settings.MaxDepthMeters)) };
        //TODO: match iteratively across each octave

        // filter frame 0 descriptors to within the crop region for matching
        std::vector<bool> flagsFrame0(frame0->GetDescriptorsCount(), false);
        int numFilteredDescriptorsFrame0 = CreateFilteredDescriptorMask(-1, frame0CropRect, frame0, flagsFrame0);
        
        std::vector<bool> flagsFrame1(frame1->GetDescriptorsCount(), true);
        int numFilteredDescriptorsFrame1 = gsl::narrow_cast<int>(frame1->GetDescriptorsCount());
        
        // do feature matching
        std::vector<cv::DMatch> matches;
        unsigned int numMatches = Match(frame0, frame1, flagsFrame0, flagsFrame1,
            numFilteredDescriptorsFrame0, numFilteredDescriptorsFrame1, m_settings.OrbMatcherSettings.MaxHammingDistance,
            m_settings.OrbMatcherSettings.MinHammingDifference, matches);

        const unsigned int minFeatureMatches = m_settings.MinFeatureMatches;

//#define DEBUG_STEREO_MATCHING
#ifdef DEBUG_STEREO_MATCHING
            cv::Mat debugMatches;

            const auto& imgAKeypoints = frame0->GetKeyPoints();
            const auto& imgBKeypoints = frame1->GetKeyPoints();
            cv::drawMatches(frame0->GetDebugImage(), std::vector<cv::KeyPoint>(imgAKeypoints.begin(), imgAKeypoints.end()),
                frame1->GetDebugImage(), std::vector<cv::KeyPoint>(imgBKeypoints.begin(), imgBKeypoints.end()),
                matches, debugMatches);

            static int j = 0;
            j++;
#endif

        LogMessage((boost::wformat(L"StereoInit: found %1% matches out of required %2%") % numMatches % minFeatureMatches).str());

        //normalized the pose
        cv::Matx44f normalizedFrame0ToFrame1(frame0ToFrame1);
        cv::Vec3f worldSpaceDelta = Translation(normalizedFrame0ToFrame1);
        float lenDelta = (float)cv::norm(worldSpaceDelta, cv::NORM_L2);
        const float minPoseDelta = 0.00001f;
        if (lenDelta < minPoseDelta)
        {
            LogMessage(L"StereoInit: [FAILURE] zero displacement between frames");
            return boost::none;
        }

        float oneOverMeters = 1 / lenDelta;
        normalizedFrame0ToFrame1(0, 3) *= oneOverMeters;
        normalizedFrame0ToFrame1(1, 3) *= oneOverMeters;
        normalizedFrame0ToFrame1(2, 3) *= oneOverMeters;

        // triangulate points      
        if (numMatches < minFeatureMatches)
        {
            LogMessage(L"StereoInit: [FAILURE] not enough feature matches");
            return boost::none;
        }
            
        Pose normalizedframe1Pose = { normalizedFrame0ToFrame1.inv() }; //initializer takes a world matrix which is the inverse of wide to narrow
        InitializationData initializationData;
        InitializeWithFrames(matches, frame0, frame1, initializationData, normalizedframe1Pose, m_settings.MaxEpipolarError, m_settings.MinAcceptedDistanceRatio, memory);

        if (initializationData.MapPoints.size() < m_settings.MinInitMapPoints)
        {
            LogMessage((boost::wformat(L"StereoInit: [FAILURE] not enough map points with: %1%") % initializationData.MapPoints.size()).str());

            return boost::none;
        }

        initializationData.Frames.back()->AddExtrinsicTether(initializationData.Frames.front()->GetId(), initializationData.Frames.front()->GetPose(), m_settings.InitializationTetherStrength);

        // perform an initial bundle adjust of the map before creating new map points
        MapInitialization::BundlerSettings bundlerSettings{
            m_settings.BundleAdjustSettings.HuberWidth,
            m_settings.BundleAdjustSettings.MaxOutlierError,
            m_settings.BundleAdjustSettings.MaxOutlierErrorScaleFactor,
            m_settings.BundleAdjustSettings.MinMeanSquareError,
            false,
            m_settings.BundleAdjustSettings.NumStepsPerRun,
            m_settings.BundleAdjustSettings.NumSteps,
            m_settings.BundleAdjustSettings.MinSteps };

        MapInitialization::BundleAdjustInitializationData(initializationData, m_determinator, true, bundlerSettings, memory);

        if (!MapInitialization::ValidateInitializationData(initializationData,
            m_settings.MaxPoseContributionZ,
            m_settings.AmountBACanChangePose,
            m_settings.MinInitMapPoints)
            )
        {
            LogMessage((boost::wformat(L"StereoInit: [FAILURE] init failed validation")).str());
            return boost::none;
        }

        return initializationData;
    }
}
