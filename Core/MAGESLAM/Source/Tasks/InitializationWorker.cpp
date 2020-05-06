// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "InitializationWorker.h"

#include "Tracking/MapInitialization.h"
#include "Tracking/BoundingPlaneDepths.h"

#include "Utils/Logging.h"
#include "Utils/MageConversions.h"

#include "MageContext.h"

namespace mage
{
    struct InitializationWorker::Impl
    {
        mira::dispatcher<72>& Dispatcher;
        mira::determinator& Determinator;
        MageContext& Context;
        const MageSlamSettings& Settings;

        std::unique_ptr<MapInitialization> Init{};

        Impl(mira::dispatcher<72>& dispatcher, mira::determinator& determinator, MageContext& context, const MageSlamSettings& settings)
            : Dispatcher{ dispatcher }
            , Determinator{ determinator }
            , Context{ context }
            , Settings{ settings }
        {}
    };

    InitializationWorker::InitializationWorker(
        mira::dispatcher<72>& dispatcher,
        mira::determinator& determinator,
        MageContext& context,
        const MageSlamSettings& settings)
        :   BaseWorker(100 * 1024, 10 * 1000 * 1024),
            m_impl{ std::make_unique<Impl>(dispatcher, determinator, context, settings) }
    {
        m_impl->Init = std::make_unique<MapInitialization>(settings.MonoSettings.MonoMapInitializationSettings, GetSettingsForCamera(settings, mage::CameraIdentity::MONO), context.IMUCharacterization, determinator);
    }

    mira::task<PoseRefined> InitializationWorker::Initialize(const FrameAnalyzed& frame)
    {
        return mira::make_task(m_impl->Dispatcher, Cancellation(), [this, frame]() -> mira::expected<PoseRefined>
        {
            thread_memory memory = MemoryPool().create();

            InitializationData initializationData;

            auto initAttempt = m_impl->Init->TryInitializeMap(
                frame.Analyzed, memory, initializationData, m_impl->Context.BagOfWords);

            DETERMINISTIC_CHECK(m_impl->Determinator, initAttempt);

            bool initialized = initAttempt == MapInitialization::InitializationAttemptState::FinishInit;
            if (!initialized)
            {
                return mira::errc::not_enough_input;
            }

            DETERMINISTIC_CHECK(m_impl->Determinator, initializationData.MapPoints);
            DETERMINISTIC_CHECK(m_impl->Determinator, initializationData.Frames);

            m_impl->Context.Introspection.Introspect(initializationData);

            m_impl->Context.Map.InitializeMap(initializationData, memory);

            // when we initialize our map with keyframes we need to add the keyframes to the bow and the pose
            // history, because they weren't added from the mapping thread like the regular pipeline
            for (const auto& builder : initializationData.Frames)
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

            return PoseRefined{ false, initializationData.Frames.back(), {} };
        });
    }

    InitializationWorker::~InitializationWorker() = default;
}
