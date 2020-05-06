// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once
#include "Data\Intrinsics.h"
#include "Data\Pose.h"
#include "Device/CameraCalibration.h"
#include "MageSettings.h"
#include "Device\IMUCharacterization.h"

#include "Map\Map.h"
#include "Map\ThreadSafeMap.h"
#include "Bow\BaseBow.h"

#include "Image\AnalyzedImage.h"

#include "Proxies\MapPointProxy.h"

#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <memory>
#include <gsl\gsl>

namespace UnitTests
{
    class MapInitializationUnitTest;
}

namespace mage
{
    class KeyframeBuilder;

    class MapInitialization
    {
    public:

        enum InitializationAttemptState
        {
            NoPose,
            ResetInit,
            FinishInit,
            Skipped
        };

        MapInitialization(const MonoMapInitializationSettings& settings, const PerCameraSettings& cameraSettings, const device::IMUCharacterization& imuCharacterization, mira::determinator& determinator);

        InitializationAttemptState TryInitializeMap(const std::shared_ptr<const AnalyzedImage>& frame,
            thread_memory memory,
            InitializationData& initializationData,
            BaseBow& bagOfWords);

        bool InitializeWithFrames(gsl::span<const cv::DMatch> matches,
            const std::shared_ptr<const AnalyzedImage>& frame0,
            const std::shared_ptr<const AnalyzedImage>& frame1,
            InitializationData& initializationData,
            thread_memory memory);

        struct BundlerSettings
        {
            float HuberWidth;
            float MaxOutlierError;
            float MaxOutlierErrorScaleFactor;
            float MinMeanSquareError;
            bool FixMapPoints;
            uint32_t NumStepsPerRun;
            uint32_t NumSteps;
            uint32_t MinSteps;
        };
        
        static void BundleAdjustInitializationData(InitializationData& initializationData, mira::determinator& determinator, bool cullOutliers, const BundlerSettings& bundlerSettings, thread_memory memory);
        static bool ValidateInitializationData(const InitializationData& initializationData, float maxZContribution, float amountBACanChangePose, size_t minFeatureCount);

        // Frame1Points and Frame2Points are indexed to match each other
        static void CollectMatchPoints(const std::shared_ptr<const AnalyzedImage>& referenceFrame,
                const std::shared_ptr<const AnalyzedImage>& currentFrame,
                gsl::span<const cv::DMatch> matches,
                std::vector<cv::Point2f>& frame1Points,
                std::vector<cv::Point2f>& frame2Points);

        static void TriangulatePoints(const std::shared_ptr<const AnalyzedImage>& referenceFrame, const std::shared_ptr<const AnalyzedImage>& currentFrame,
            const Pose & referencePose, const Pose & currentPose,
            gsl::span<const cv::DMatch> matches,
            gsl::span<const cv::Point2f> frame1_points, gsl::span<const cv::Point2f> frame2_points,
            float pixelMaxEpipolarDistance, float minAcceptanceDistanceRatio,
            std::vector<std::pair<cv::DMatch, cv::Point3f>>& initial3DPoints);

    private:

        struct MatchedImage
        {
            std::shared_ptr<const AnalyzedImage>    Image;
            std::vector<cv::DMatch>                 Matches;

            MatchedImage(std::shared_ptr<const AnalyzedImage> image, std::vector<cv::DMatch> matches)
                : Image{ image },
                Matches( std::move(matches) )
            {};

            MatchedImage(std::shared_ptr<const AnalyzedImage> image)
                : Image{ image }
            {};
        };

        struct PointAssociation
        {
            size_t      PointIndex2d;
            size_t      PointIndex3d;

            PointAssociation(size_t twoD, size_t threeD)
                : PointIndex2d { twoD },
                PointIndex3d { threeD }
            {};
        };

        struct InitializationPose
        {
            std::shared_ptr<const AnalyzedImage>    Image;
            mage::Pose                              Pose;
            std::vector<PointAssociation>           Associations;

            InitializationPose(std::shared_ptr<const AnalyzedImage> image, mage::Pose pose, std::vector<PointAssociation> associations)
                : Image( image ),
                Pose( pose ),
                Associations(std::move(associations))
            {};
        };

        void ResetMapInitialization();

        bool TryIntializeMapWithProvidedFrames(
            const MatchedImage& referenceFrame,
            MatchedImage& currentFrame,
            thread_memory memory,
            InitializationData& initializationData,
            BaseBow& bagOfWords);

        std::vector<Pose> FindEssentialPotientialPoses(
            const cv::Matx33f& essentialMat);
        
        //Frame1Points and Frame2Points are indexed to match each other
        float ScoreFundamentalMatrix(gsl::span<const cv::Point2f> frame1Points,
            gsl::span<const cv::Point2f> frame2Points,
            const cv::Matx33f& fundamentalMat1To2,
            const cv::Matx33f& fundamentalMat2To1);

        //Frame1Points and Frame2Points are indexed to match each other
        std::vector<Pose> FindPossiblePoses(gsl::span<const cv::Point2f> frame1Points,
            gsl::span<const cv::Point2f> frame2Points,
            const CameraCalibration& frame1Calibration,
            const CameraCalibration& frame2Calibration);

        //Frame1Points and Frame2Points are indexed to match each other
        bool FindCorrectPose(gsl::span<const cv::Point2f> frame1Points,
            gsl::span<const cv::Point2f> frame2Points,
            gsl::span<const cv::DMatch> matches,
            const CameraCalibration& frame1Calibration,
            const CameraCalibration& frame2Calibration,
            const Pose& referencePose,
            const std::vector<Pose>& poses,
            Pose& correctPose,
            std::vector<std::pair<cv::DMatch, cv::Point3f>>& correct3DPoints);

        const MonoMapInitializationSettings m_settings;
        const PerCameraSettings m_cameraSettings;
        const device::IMUCharacterization& m_imuCharacterization;
        mira::determinator& m_determinator;

        std::vector<MatchedImage> m_initializationFrames;
        std::vector<uint8_t> m_initializationDescriptorsCounters;

        //For Test
        friend class ::UnitTests::MapInitializationUnitTest;
     };
}
