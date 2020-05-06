// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "TrackLocalMapWorker.h"

#include "Map/ThreadSafePoseHistory.h"

#include "Utils/Logging.h"

#include "MageContext.h"
#include "Map/ThreadSafeMap.h"
#include "LoopClosureWorker.h"
#include "Tracking/TrackLocalMap.h"
#include "Analysis/binary_iterators.h"
#include "Tracking/BoundingPlaneDepths.h"
#include "Tracking/NewKeyFrameDecision.h"

#include "Debugging/SkeletonLogger.h"

#include <arcana/threading/pending_task_scope.h>

namespace mage
{
    namespace
    {
        struct MapPointMergeInfo
        {
            Id<MapPoint> OriginalId{};
            const MapPointTrackingProxy* NewMapPointProxy{ nullptr };
            KeypointDescriptorIndex DescriptorIndex{};

            MapPointMergeInfo() = default;
            MapPointMergeInfo(Id<MapPoint> id, const MapPointTrackingProxy* proxy, KeypointDescriptorIndex idx)
                : OriginalId{ id },
                  NewMapPointProxy{ proxy },
                  DescriptorIndex{ idx }
            {}
        };
    }

    struct TrackLocalMapWorker::Impl
    {
        TrackingMediator& Mediator;
        mira::determinator& Determinator;
        MageContext& Context;
        const MageSlamSettings& Settings;

        TrackingFrameHistory TrackingHistory{};
        std::unique_ptr<TrackLocalMap> TrackLocalMap;
        std::unique_ptr<NewKeyFrameDecision> NewKeyFrameDecision;

        bool MappingIdle = true;
        bool MappingWillBeIdle = true;

        bool JustInsertedAKeyframe = false;

        std::shared_ptr<std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>> MapPointMerges{};

        struct AlgorithmData
        {
            std::shared_ptr<const PoseEstimated> PoseEstimation;

            // read data from the map in a synchronization block to make the algorithm deterministic
            std::map<Id<MapPoint>, unsigned int> RecentlyCreatedMapPoints;

            boost::optional<loop::vector<KeyframeReprojection>> ConnectedKeyframes;
        } Data{};
    };

    TrackLocalMapWorker::TrackLocalMapWorker(
        gsl::span<KeyframeProxy> keyframes,
        TrackingMediator& mediator,
        mira::determinator& determinator,
        MageContext& context,
        const MageSlamSettings& settings)
        :   BaseWorker{ 100 * 1024, 10 * 1000 * 1024 },
            m_impl{ std::unique_ptr<Impl>(new Impl{ mediator, determinator, context, settings }) }
    {
        for (auto& keyframe : keyframes)
            m_impl->TrackingHistory.advance(HistoricalFrame{ std::make_shared<KeyframeProxy>(keyframe) });

        mediator.send(HistoryUpdated{ m_impl->TrackingHistory });

        m_impl->TrackLocalMap = std::make_unique<TrackLocalMap>(settings, context.IMUCharacterization, determinator);
        m_impl->NewKeyFrameDecision = std::make_unique<NewKeyFrameDecision>(settings);

        Registrations() += mediator.add_listener<TrackingLost>([&](const TrackingLost&)
        {
            m_impl->TrackingHistory.clear();
        });
    }

    const TrackingFrameHistory& TrackLocalMapWorker::GetHistory() const
    {
        return m_impl->TrackingHistory;
    }

    mira::task<PoseRefined> TrackLocalMapWorker::RefinePose(const std::shared_ptr<const PoseEstimated>& estimated)
    {
        m_impl->Data.PoseEstimation = estimated;

        return m_impl->Context.StateMachine.on(TrackingReadState, m_impl->Mediator.dispatcher(), Cancellation(), [this](bool& keyframeInserted)
        {
            SCOPE_TIMER(TrackLocalMapWorker::TrackingRead);

            keyframeInserted = m_impl->JustInsertedAKeyframe;
            m_impl->JustInsertedAKeyframe = false;

            if (m_impl->MappingWillBeIdle)
            {
                m_impl->MappingIdle = true;
                m_impl->MappingWillBeIdle = false;
            }

            auto memory = MemoryPool().create();

            auto mapPoints = memory.stack_vector<MapPointTrackingProxy>();
            m_impl->Data.PoseEstimation->Frame->GetMapPoints(mapPoints);

            m_impl->Data.ConnectedKeyframes = memory.loop_vector<KeyframeReprojection>();
            m_impl->Context.Map.GetConnectedMapPoints(mapPoints, memory, *m_impl->Data.ConnectedKeyframes);

            DETERMINISTIC_CHECK(m_impl->Determinator, mapPoints.begin(), mapPoints.end());
            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->Data.ConnectedKeyframes->begin(), m_impl->Data.ConnectedKeyframes->end());

            m_impl->Context.History.FlushTemporaryPoseHistory();

            for (auto& historicalFrame : m_impl->TrackingHistory)
            {
                DETERMINISTIC_CHECK(m_impl->Determinator, historicalFrame.UpdatedPose);

                auto maybePose = m_impl->Context.History.GetPoseForFrame(historicalFrame.Keyframe->GetAnalyzedImage()->GetFrameId());
                if (maybePose.is_initialized())
                {
                    historicalFrame.UpdatedPose = maybePose.value();
                    DETERMINISTIC_CHECK(m_impl->Determinator, historicalFrame.UpdatedPose);
                }

                if (m_impl->MapPointMerges != nullptr)
                {
                    std::vector<MapPointMergeInfo> merged;
                    merged.reserve(historicalFrame.Keyframe->GetMapPointCount());

                    historicalFrame.Keyframe->IterateAssociations([&](MapPointTrackingProxy& mp, KeypointDescriptorIndex idx)
                    {
                        const auto& mergePtr = m_impl->MapPointMerges->find(mp.GetId());
                        if (mergePtr != m_impl->MapPointMerges->end())
                        {
                            merged.emplace_back(mergePtr->first, &mergePtr->second, idx);
                        }
                    });

                    for (const auto& mp : merged)
                    {
                        historicalFrame.Keyframe->RemoveAssociation(mp.OriginalId);

                        if (!historicalFrame.Keyframe->HasMapPoint(mp.NewMapPointProxy->GetId()))
                        {
                            historicalFrame.Keyframe->AddAssociation(*mp.NewMapPointProxy, mp.DescriptorIndex);
                        }
                    }
                }

                m_impl->Context.Map.UpdateMapPoints(*historicalFrame.Keyframe);
            }

            m_impl->MapPointMerges.reset();

            m_impl->Data.RecentlyCreatedMapPoints = m_impl->Context.Map.GetRecentlyCreatedMapPoints();
            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->Data.RecentlyCreatedMapPoints.begin(), m_impl->Data.RecentlyCreatedMapPoints.end());

            // Apply the relative pose.
            boost::optional<Pose> basisPoseFromHistory = m_impl->Context.History.GetPoseForFrame(m_impl->Data.PoseEstimation->BasisFrameId);
            assert(basisPoseFromHistory);
            auto updatedPose = m_impl->TrackLocalMap->GetUpdatedKeyframePoseRelativeToBasis(
                m_impl->Data.PoseEstimation->Frame->GetPose(),
                m_impl->Data.PoseEstimation->BasisPose,
                *basisPoseFromHistory
                );
            m_impl->Data.PoseEstimation->Frame->SetPose(updatedPose);
        }).then(m_impl->Mediator.dispatcher(), Cancellation(), [this]()
        {
            SCOPE_TIMER(TrackLocalMapWorker::RunTrackLocalMap);

            auto memory = MemoryPool().reuse();

            if (!m_impl->TrackLocalMap->RunTrackLocalMap(*m_impl->Data.ConnectedKeyframes,
                                                         std::move(m_impl->Data.RecentlyCreatedMapPoints),
                                                         memory,
                                                         true,
                                                         m_impl->Settings.TrackLocalMapSettings.UnassociateOutliers,
                                                         *m_impl->Data.PoseEstimation->Frame))
            {
                return mira::expected<PoseRefined>{ mira::errc::failed };
            }

            // BUG: make sure the pose estimation waits for the updated history
            m_impl->TrackingHistory.advance(HistoricalFrame{ m_impl->Data.PoseEstimation->Frame });
            m_impl->Mediator.send(HistoryUpdated{ m_impl->TrackingHistory });

            // new keyframe decision
            InternalDepth depth = CalculateBoundingPlaneDepthsForKeyframe(*m_impl->Data.PoseEstimation->Frame, m_impl->Settings.BoundingDepthSettings);
            SkeletonLogger::TrackingThread::LogBoundingPlaneDepths(*m_impl->Data.PoseEstimation->Frame, depth.AsDepth());
            m_impl->Context.History.AddHistoricalPose(*m_impl->Data.PoseEstimation->Frame, *m_impl->Data.ConnectedKeyframes, std::move(depth), memory);

            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->Data.PoseEstimation->Frame);

            auto mapPoints = memory.stack_vector<MapPointTrackingProxy>();
            m_impl->Data.PoseEstimation->Frame->GetMapPoints(mapPoints);

            temp::vector<std::reference_wrapper<const KeyframeReprojection>> connectedKeyframesRef =
                memory.stack_vector<std::reference_wrapper<const KeyframeReprojection>>(m_impl->Data.ConnectedKeyframes->size());

            connectedKeyframesRef.insert(connectedKeyframesRef.end(),
                m_impl->Data.ConnectedKeyframes->begin(), m_impl->Data.ConnectedKeyframes->end());

            // Try to insert frame as new KeyFrame
            bool shouldInsertNewKeyframe = m_impl->NewKeyFrameDecision->IsNewKeyFrame(
                m_impl->MappingIdle,
                *m_impl->Data.PoseEstimation->Frame,
                connectedKeyframesRef,
                mapPoints,
                m_impl->Data.PoseEstimation->UsingRelocalization,
                depth.NearPlaneDepth);

            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->Data.PoseEstimation->Frame->GetMapPointCount());
            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->Data.PoseEstimation->UsingRelocalization);

            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->MappingIdle);
            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->Data.ConnectedKeyframes->begin(), m_impl->Data.ConnectedKeyframes->end());

            if (shouldInsertNewKeyframe)
            {
                m_impl->MappingIdle = false;
                m_impl->JustInsertedAKeyframe = true;
                return mira::expected<PoseRefined>{ PoseRefined{ true, m_impl->Data.PoseEstimation->Frame, m_impl->TrackLocalMap->GetRecentMapPointsThatFailScoring() } };
            }
            else
            {
                return mira::expected<PoseRefined>{ PoseRefined{ false, m_impl->Data.PoseEstimation->Frame, {} } };
            }
        }).then(mira::inline_scheduler, mira::cancellation::none(), [this](const mira::expected<PoseRefined>& refined)
        {
            // clear all the data for the current frame.
            // do this no matter if the run is cancelled/completed or error'd.
            m_impl->Data = {};

            m_impl->Context.TickImageFactories(MageContext::ThreadType::TrackingThread);

            return refined;
        });
    }

    void TrackLocalMapWorker::SetMappingIdle()
    {
        m_impl->MappingWillBeIdle = true;
    }

    void TrackLocalMapWorker::ConsumeUpdateFromLoopClosure(const LoopClosureTrackingUpdate& trackingUpdate)
    {
        assert(m_impl->MapPointMerges == nullptr);
        m_impl->MapPointMerges = trackingUpdate.MapPointMerges;
    }

    TrackLocalMapWorker::~TrackLocalMapWorker() = default;
}
