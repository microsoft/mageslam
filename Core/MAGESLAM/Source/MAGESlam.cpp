// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MageSlam.h"

#include "Utils/Logging.h"

#include "Map/ThreadSafeMap.h"
#include "Debugging/SkeletonData.h"
#include "Utils/MageConversions.h"

#include "BoW/BaseBow.h"

#include "Image/ImageFactory.h"
#include "Device/Device.h"
#include "Fuser/Fuser.h"
#include "Debugging/SkeletonLogger.h"
#include "Analysis/DataFlow.h"

#include "Platform/Platform.h"

#include "Tasks/Runtime.h"

#include "Tasks/ImageAnalyzer.h"
#include "Tasks/InitializationWorker.h"
#include "Tasks/FuserWorker.h"
#include "Tasks/PoseEstimationWorker.h"
#include "Tasks/TrackLocalMapWorker.h"
#include "Tasks/MappingWorker.h"

#include "Tracking/BoundingPlaneDepths.h"

using namespace std;

namespace
{
    const uint32_t TARGET_FRAME_RATE = 30;
}

namespace mage
{
    struct MAGESlam::Impl
    {
        const MageSlamSettings m_settings;
        const device::IMUCharacterization m_imuCharacterization;

        ImageFactoryMap m_imageFactoryMap;
        std::vector<CameraConfiguration> m_cameraConfigurations;

        mira::cancellation_source::ticket_scope m_scopes;

        mira::state_machine_driver m_driver;

        std::unique_ptr<BaseBow> m_bow;
        std::unique_ptr<ThreadSafeMap> m_map;
        std::unique_ptr<ThreadSafePoseHistory> m_poseHistory;
        std::unique_ptr<Fuser> m_fuser;

        std::unique_ptr<MageContext> m_context;

        std::unique_ptr<Runtime> m_runtime;
       
        Impl(const MageSlamSettings& settings, gsl::span<const CameraConfiguration> cameras, const device::IMUCharacterization& imuCharacterization)
            :   m_settings{ settings },
                m_imuCharacterization{ imuCharacterization },
                m_cameraConfigurations{ cameras.begin(), cameras.end() },
                m_poseHistory{ make_unique<ThreadSafePoseHistory>(m_settings.PoseHistorySettings) }
        {
            assert(!cameras.empty() && "Must Specify at least one camera configuration");
            //TODO figure out what to do with stereo BoW?  Initial guess is to treat the pairs as a single entity
            const PerCameraSettings& firstCameraSettings = GetSettingsForCamera(m_settings, cameras.begin()->CameraIdentity);

            m_bow = make_unique<OnlineBow>(m_settings.BagOfWordsSettings, firstCameraSettings.FeatureExtractorSettings.NumFeatures);
            m_fuser = make_unique<Fuser>(m_settings.FuserSettings, m_imuCharacterization, m_settings.PoseEstimationSettings);

            for (const auto& cameraConfiguration : m_cameraConfigurations)
            {
                const PerCameraSettings& pcs = GetSettingsForCamera(m_settings, cameraConfiguration.CameraIdentity);
                std::unique_ptr<ImageFactory> factory;
                
                factory = std::make_unique<ImageFactory>(
                    cameraConfiguration.CameraIdentity,
                    pcs.FeatureExtractorSettings.NumFeatures,
                    pcs.FeatureExtractorSettings.ScaleFactor,
                    pcs.FeatureExtractorSettings.NumLevels,
                    pcs.FeatureExtractorSettings.GetImageBorder(),
                    100);// pre-allocate room for 100 images

                m_imageFactoryMap.emplace(cameraConfiguration.CameraIdentity, std::move(factory));
            }

            m_map = make_unique<ThreadSafeMap>(m_settings, *m_bow);

            m_context = std::make_unique<MageContext>(m_imageFactoryMap, *m_bow, *m_map, *m_poseHistory, m_imuCharacterization, m_driver);

#if PROFILE_MEMORY
            platform::profile_memory();
#endif

            m_runtime = std::make_unique<Runtime>(m_settings, *m_context, *m_fuser, m_driver);

            m_runtime->Run(m_cameraConfigurations);
        }
        
        void CreateImage(const MAGESlam::Frame& frame, ImageHandle& imageData, cv::Mat& imageMat)
        {
            SCOPE_TIMER(MAGESlam::CreateImage);

            // get the ImageFactory which matches this frame's format
            assert(m_imageFactoryMap.find(frame.Format.FrameId.Camera) != m_imageFactoryMap.end());
            ImageFactory& factory = *m_imageFactoryMap.at(frame.Format.FrameId.Camera);
            factory.CleanupImages();
            imageData = factory.AllocateMetaData();

            // get the configuration for the correct camera
            const auto& config = std::find_if(m_cameraConfigurations.begin(), m_cameraConfigurations.end(),
                [&frame](const CameraConfiguration& cameraConfiguration)
            {
                return cameraConfiguration.CameraIdentity == frame.Format.FrameId.Camera;
            });
            assert(config != m_cameraConfigurations.end());

            imageMat = CreateGrayCVMat(cv::Size{ (int)config->Size.Width, (int)config->Size.Height }, config->Format, frame.Bytes);
        }

        ~Impl()
        {
            m_runtime.reset();
        }
    };

    function<SkeletonData (const MAGESlam&)> g_backdoor;
    function<FossilizedSkeletonData (const MAGESlam::FossilizedMap&)> g_fossilizedBackdoor;

    MAGESlam::MAGESlam(const MageSlamSettings& settings, gsl::span<const CameraConfiguration> cameraConfigurations, const device::IMUCharacterization& imuCharacterization)
        : m_impl{ std::make_unique<Impl>(settings, cameraConfigurations, imuCharacterization) }
    {
        // validate that the settings have been overridden because we EXPECT that to be true
        if (!settings.Metadata.LoadedFromFile)
        {
            std::cerr << "MageSlamSettings not properly specified.  You must override the default settings" << std::endl;
            throw std::logic_error("MageSlamSettings not properly overridden.  You must override the default settings");
        }

        // disable openCV parallelization
        cv::setNumThreads(0);

        IdGenerator<Keyframe>::reset();
        IdGenerator<MapPoint>::reset();

        /* backdoor used for the skeleton key */
        g_backdoor = [](const MAGESlam& slam)
        {
            return SkeletonData{
                slam.m_impl->m_map.get(),
                slam.m_impl->m_poseHistory.get(),
                slam.m_impl->m_context.get(),
                slam.m_impl->m_fuser.get()
            };
        };
    }

    MAGESlam::~MAGESlam()
    {}

    std::future<MAGESlam::Tracking> MAGESlam::ProcessFrame(const Frame& frame)
    {
        SCOPE_TIMER(MAGESlam::ProcessFrame);
     
        // Only copy the frame when the pump function says it's time to
        // send a real image to the thread.
        cv::Mat imageMat;
        ImageHandle imageData = nullptr;

        m_impl->CreateImage(frame, imageData, imageMat);

        bool addedToFilter = false;
        if (m_impl->m_settings.FuserSettings.UseFuser)
        {
            addedToFilter = m_impl->m_fuser->AddImageFence(frame.Format.Timestamp);
        }

        auto frameData = make_shared<FrameData>(frame.Format, move(imageData), imageMat, *m_impl->m_fuser, addedToFilter);

        auto future = frameData->GetFuture();

        m_impl->m_runtime->TrackMono(move(frameData));

        return future;
    }

    std::pair<std::future<MAGESlam::Tracking>, std::future<MAGESlam::Tracking>> MAGESlam::ProcessStereoFrames(const Frame& one, const Frame& two)
    {
        SCOPE_TIMER(MAGESlam::ProcessStereoFrames);

        // Only copy the frame when the pump function says it's time to
        // send a real image to the thread.
        cv::Mat imageMat1, imageMat2;
        ImageHandle imageData1 = nullptr;
        ImageHandle imageData2 = nullptr;

        m_impl->CreateImage(one, imageData1, imageMat1);
        m_impl->CreateImage(two, imageData2, imageMat2);

        bool addedToFilter1 = false;
        bool addedToFilter2 = false;
        if (m_impl->m_settings.FuserSettings.UseFuser)
        {
            if (m_impl->m_settings.StereoSettings.PrimaryTrackingCamera == CameraIdentity::STEREO_1)
            {
                addedToFilter1 = m_impl->m_fuser->AddImageFence(one.Format.Timestamp);
            }
            else
            {
                addedToFilter2 = m_impl->m_fuser->AddImageFence(two.Format.Timestamp);
            }
        }

        auto frameOne = make_shared<FrameData>(one.Format, move(imageData1), imageMat1, *m_impl->m_fuser, addedToFilter1);
        auto frameTwo = make_shared<FrameData>(two.Format, move(imageData2), imageMat2, *m_impl->m_fuser, addedToFilter2);

        auto results = std::make_pair(frameOne->GetFuture(), frameTwo->GetFuture());

        m_impl->m_runtime->TrackStereo(std::move(frameOne), std::move(frameTwo));

        return results;
    }

    vector<boost::optional<MAGESlam::TrackedFrame>> MAGESlam::GetTrackingResultsForFrames(const gsl::span<const FrameId> frameIds) const
    {
        SCOPE_TIMER(MAGESlam::GetTrackingResultsForFrames);

        auto results = m_impl->m_poseHistory->GetTrackingInformationForFrames(frameIds);
        vector<boost::optional<MAGESlam::TrackedFrame>> frames;
        for (const auto& result : results)
        {
            if (result.is_initialized())
            {
                frames.emplace_back(TrackedFrame{ Tracking{ToMageMat(result->Pose.GetViewMatrix()), TrackingState::TRACKING}, result->CameraModel, result->Depth });
            }
            else
            {
                frames.push_back(boost::none);
            }
        }

        return frames;
    }

    void MAGESlam::AddSensorSample(const SensorSample& deviceSample)
    {
        SensorSample sample = deviceSample;

        // accelerometer samples are in g's not in m/s/s
        if (sample.GetType() == mage::SensorSample::SampleType::Accelerometer)
        {
            for (float& value : sample.GetData())
            {
                // TODO BUG: The filter use 9.81f as the gravity constant, figure out if that's good or not.
                value *= g_GravityMetersPerSecPerSec;
            }
        }

        if (!m_impl->m_settings.FuserSettings.DropMagSamples || sample.GetType() != SensorSample::SampleType::Magnetometer)
        {
            m_impl->m_fuser->AddSample(sample);
        }

        m_impl->m_runtime->AddSample(sample);
    }

    bool MAGESlam::GetGravityDirection(Direction& gravDir) const
    {
        if (!m_impl->m_fuser->MapOriginValid())
            return false;

        cv::Vec3f gravityInMageWorld;
        bool goodGravity = m_impl->m_fuser->GetMageWorldGravity(gravityInMageWorld);

        gravDir.X = gravityInMageWorld[0];
        gravDir.Y = gravityInMageWorld[1];
        gravDir.Z = gravityInMageWorld[2];

        return goodGravity;
    }

    // gets the scale to apply to mage to bring it to meters as estimated by
    // the stereo tracking code
    float MAGESlam::GetStereoMageMeterEstimate() const
    {
        if (m_impl->m_cameraConfigurations.size() != 2)
            return 1.0f;

        // we're assuming that the median tether distance is the right distance
        // which means, it represents the conversion where mageUnits = extrinsicDistance
        float mageUnits = m_impl->m_map->GetMedianTetherDistance();
        if (mageUnits == 0.f)
            return 1.0f;

        cv::Matx44f cam0(&m_impl->m_cameraConfigurations[0].Extrinsics.M11);
        cv::Vec3f leftPos{ cam0.col(3).get_minor<3, 1>(0, 0).val };

        cv::Matx44f cam1(&m_impl->m_cameraConfigurations[1].Extrinsics.M11);
        cv::Vec3f rightPos{ cam1.col(3).get_minor<3, 1>(0, 0).val };

        float meters = (float)cv::norm(leftPos - rightPos, cv::NORM_L2);

        // this value times a mage unit gives meters.
        return meters / mageUnits;
    }

    bool MAGESlam::GetScaleFromIMU(float& scaleMAGEtoMeters) const
    {
        return m_impl->m_fuser->GetMageToMetersWorldScale(scaleMAGEtoMeters);
    }

    bool MAGESlam::TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest) const
    {
        return m_impl->m_poseHistory->TryGetVolumeOfInterest(volumeOfInterest, m_impl->m_settings.VolumeOfInterestSettings);
    }

    std::unique_ptr<MAGESlam::FossilizedMap> MAGESlam::Fossilize(std::unique_ptr<MAGESlam> slam)
    {
        {
            SCOPE_TIMER(MAGESlam::Fossilize::StopThreads);

            slam->m_impl->m_runtime.reset();
        }

        AdjustableData data{};

        {
            SCOPE_TIMER(MAGESlam::Fossilize::FetchData);
            slam->m_impl->m_map->BuildGlobalBundleAdjustData(data);
            std::unique_ptr<Map> map = ThreadSafeMap::Release(std::move(slam->m_impl->m_map));
        }

        bool allFixed = std::all_of(data.Keyframes.begin(), data.Keyframes.end(), [](const auto& kf) { return kf.IsFixed(); });

        // if we have no keyframes or they're all fixed,
        // there's nothing to bundle adjust
        if (!data.Keyframes.empty() && !allFixed)
        {
            auto& settings = slam->m_impl->m_settings.GraphOptimizationSettings;

            if (settings.NumSteps > 0)
            {
                SCOPE_TIMER(MAGESlam::Fossilize::BundleAdjust);

                temp_memory memory{ 100 * 1024, 100 * 1024 };

                BundleAdjustTask ba{
                    data,
                    mira::determinator::create("Fossil"),
                    settings.MaxOutlierError,
                    settings.MaxOutlierErrorScaleFactor,
                    false,
                    settings.NumSteps
                };

                ba.IterateBundleAdjust(settings.BundleAdjustmentHuberWidth);
            }
        }

        // be sure to save any buffered state in the history before releasing it
        slam->m_impl->m_poseHistory->FlushTemporaryPoseHistory();
        std::unique_ptr<PoseHistory> history = ThreadSafePoseHistory::Release(std::move(slam->m_impl->m_poseHistory));
        for (const auto& kf : data.Keyframes)
        {
            history->UpdateKeyframePose(kf.GetId(), kf.GetPose());
        }

        std::vector<Position> mapPoints;
        mapPoints.reserve(data.MapPoints.size());
        for (const auto& mp : data.MapPoints)
        {
            mapPoints.push_back(Position{ mp.GetPosition().x, mp.GetPosition().y, mp.GetPosition().z });
        }

        mage::SkeletonLogger::UpdatedPose::LogUpdatedPose(*history);

        return std::unique_ptr<FossilizedMap>{ new FossilizedMap(std::move(history), std::move(mapPoints), slam->m_impl->m_settings) };
    }

    struct MAGESlam::FossilizedMap::Impl
    {
        std::unique_ptr<const PoseHistory> History;
        std::vector<Position> MapPoints;
        const MageSlamSettings Settings;

        Impl(std::unique_ptr<const PoseHistory> history, std::vector<Position> mapPoints, const MageSlamSettings& settings)
            : History{ std::move(history) },
            MapPoints{ std::move(mapPoints) },
            Settings{ settings }
        {}
    };

    MAGESlam::FossilizedMap::FossilizedMap(std::unique_ptr<const PoseHistory> history, std::vector<Position> mapPoints, const MageSlamSettings& settings)
        : m_impl{ std::make_unique<Impl>(std::move(history), std::move(mapPoints), settings) }
    {
        /* backdoor used for the skeleton key */
        g_fossilizedBackdoor = [](const MAGESlam::FossilizedMap& slam)
        {
            return FossilizedSkeletonData{
                slam.m_impl->History.get(),
                slam.m_impl->MapPoints
            };
        };
    }

    vector<boost::optional<MAGESlam::TrackedFrame>> MAGESlam::FossilizedMap::GetTrackingResultsForFrames(const gsl::span<const FrameId> frameIds) const
    {
        SCOPE_TIMER(MAGESlam::FossilizedMap::GetTrackingResultsForFrames);

        vector<boost::optional<TrackedFrame>> frames;
        frames.reserve(frameIds.size());

        for (const FrameId& frameId : frameIds)
        {
            auto trackingInformation = m_impl->History->GetTrackingInformationForFrame(frameId);

            if (trackingInformation.is_initialized())
            {
                frames.emplace_back(TrackedFrame{ Tracking{ ToMageMat(trackingInformation->Pose.GetViewMatrix()), TrackingState::TRACKING }, trackingInformation->CameraModel, trackingInformation->Depth });
            }
            else
            {
                frames.push_back(boost::none);
            }
        }

        return frames;
    }

    bool MAGESlam::FossilizedMap::TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest) const
    {
        return m_impl->History->TryGetVolumeOfInterest(volumeOfInterest, m_impl->Settings.VolumeOfInterestSettings);
    }

    MAGESlam::FossilizedMap::~FossilizedMap()
    {}
}
