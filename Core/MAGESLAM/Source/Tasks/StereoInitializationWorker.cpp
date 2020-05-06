// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "StereoInitializationWorker.h"

#include "Stereo/StereoMapInit.h"
#include "Tracking/BoundingPlaneDepths.h"

#include "Utils/Logging.h"
#include "Utils/MageConversions.h"
#include "Utils/CameraConfiguration.h"

#include "MageContext.h"

namespace mage
{
    struct StereoInitializationWorker::Impl
    {
        mira::dispatcher<72>& Dispatcher;
        mira::determinator& Determinator;
        MageContext& Context;
        const MageSlamSettings& Settings;

        std::vector<MAGESlam::CameraConfiguration> CameraConfigurations;
        std::unique_ptr<StereoMapInit> Tracker{};

        Impl(mira::dispatcher<72>& dispatcher,
            mira::determinator& determinator,
            gsl::span<const MAGESlam::CameraConfiguration> cameraConfigurations,
            MageContext& context,
            const MageSlamSettings& settings)
            : Dispatcher{ dispatcher }
            , Determinator{ determinator }
            , Context{ context }
            , Settings{ settings }
            , CameraConfigurations( cameraConfigurations.begin(), cameraConfigurations.end() )
        {}
    };

    StereoInitializationWorker::StereoInitializationWorker(
        mira::dispatcher<72>& dispatcher,
        mira::determinator& determinator,
        gsl::span<const MAGESlam::CameraConfiguration> cameraConfigurations,
        MageContext& context,
        const MageSlamSettings& settings)
        :   BaseWorker(100 * 1024, 10 * 1000 * 1024),
            m_impl{ std::make_unique<Impl>(dispatcher, determinator, cameraConfigurations, context, settings) }
    {
        m_impl->Tracker = std::make_unique<StereoMapInit>(settings.StereoSettings.StereoMapInitializationSettings, determinator);
    }

    mira::task<PoseRefined> StereoInitializationWorker::Initialize(const StereoFramesAnalyzed& frames)
    {
        return mira::make_task(m_impl->Dispatcher, Cancellation(), [this, frames]() -> mira::expected<PoseRefined>
        {
            thread_memory memory = MemoryPool().create();

            cv::Matx44f panelOriginToWide = ToCVMat4x4(FindCameraConfiguration(CameraIdentity::STEREO_1, m_impl->CameraConfigurations).Extrinsics);
            cv::Matx44f panelOriginToNarrow = ToCVMat4x4(FindCameraConfiguration(CameraIdentity::STEREO_2, m_impl->CameraConfigurations).Extrinsics);
            cv::Matx44f wfovToRGB = panelOriginToNarrow * panelOriginToWide.inv();

            auto initializationData = m_impl->Tracker->Initialize(frames.One.Analyzed, frames.Two.Analyzed, wfovToRGB, memory);

            bool initialized = initializationData.is_initialized();
            if (!initialized)
            {
                return mira::errc::not_enough_input;
            }

            DETERMINISTIC_CHECK(m_impl->Determinator, initializationData->MapPoints);
            DETERMINISTIC_CHECK(m_impl->Determinator, initializationData->Frames);

            m_impl->Context.Introspection.Introspect(*initializationData);

            m_impl->Context.Map.InitializeMap(*initializationData, memory);

            // when we initialize our map with keyframes we need to add the keyframes to the bow and the pose
            // history, because they weren't added from the mapping thread like the regular pipeline
            for (const auto& builder : initializationData->Frames)
            {
                m_impl->Context.BagOfWords.AddImage(builder->GetId(), *builder->GetAnalyzedImage());

                loop::vector<KeyframeReprojection> connectedKeyframes = memory.loop_vector<KeyframeReprojection>();
                temp::vector<MapPointTrackingProxy> mapPoints = memory.stack_vector<MapPointTrackingProxy>();
                builder->GetMapPoints(mapPoints);
                m_impl->Context.Map.GetConnectedMapPoints(mapPoints, memory, connectedKeyframes);

                m_impl->Context.History.AddKeyframeToTrackingHistory(builder->GetId(), builder->GetPose());
                m_impl->Context.History.AddHistoricalPose(
                    *builder, connectedKeyframes,
                    CalculateBoundingPlaneDepthsForKeyframe(*builder, m_impl->Settings.BoundingDepthSettings),
                    memory);
            }

            m_impl->Tracker.reset();
            Registrations().clear();

            // get the tracking frame that just entered the pipeline and use that as the pose
            auto frame = std::find_if(initializationData->Frames.rbegin(), initializationData->Frames.rend(), [&](const std::shared_ptr<KeyframeBuilder>& kf)
            {
                return kf->GetAnalyzedImage()->GetFrameId().Camera == m_impl->Settings.StereoSettings.PrimaryTrackingCamera;
            });

            return PoseRefined{ false, *frame, {} };
        });
    }

    StereoInitializationWorker::~StereoInitializationWorker() = default;
}
