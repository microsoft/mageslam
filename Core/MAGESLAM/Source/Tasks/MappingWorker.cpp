// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MappingWorker.h"

#include "Tracking/TrackLocalMap.h"
#include "Mapping/NewMapPointsCreation.h"
#include "Analysis/binary_iterators.h"

#include "Platform/Platform.h"

#include "Utils/Logging.h"
#include <Analysis/DataPoints.h>
#include <arcana/analysis/object_trace.h>

namespace mage
{
    namespace
    {
        std::vector<MapPointAssociations<MapPointTrackingProxy>::Association> CheapLoopClosure(ThreadSafeMap& map, const MageSlamSettings& settings, const KeyframeProxy& keyFrameproxy, int loopCounter, thread_memory memory)
        {
            if (map.GetKeyframesCount() < settings.LoopClosureSettings.MinKeyframe)
            {
                return {};
            }

            //TODO: this version uses simple sampling of map and project those into current frame.
            //      other ideas are keyframes near this frame in space or BOW.
            //      current method also use co-visable frames which have already had a chance to be matched
            //      in Track local map. This will reconnect some that have been discarded.
            //      Will experiment with this during "make tracking better" task.
            SCOPE_TIMER(Scheduling::CheapLoopClosure);
            auto mapPoints = memory.stack_vector<MapPointProxy>();
            map.GetSampleOfMapPoints(mapPoints, settings.LoopClosureSettings.MaxMapPoints, loopCounter);

            const cv::Matx33f& cameraCalibrationMatrix = keyFrameproxy.GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix();
            const Pose& pose = keyFrameproxy.GetPose();
            const cv::Matx34f& viewMatrix = pose.GetViewMatrix();
            const cv::Point3f framePosition = pose.GetWorldSpacePosition();
            const cv::Vec3f frameForward = pose.GetWorldSpaceForward();
            std::vector<bool> unassociatedMask = keyFrameproxy.GetUnassociatedKeypointMask();

            const float imageBorder = keyFrameproxy.GetAnalyzedImage()->GetImageBorder() - settings.LoopClosureSettings.MatchSearchRadius / 2.0f;
            const float pyramidScale = keyFrameproxy.GetAnalyzedImage()->GetPyramidScale();
            const size_t numLevels = keyFrameproxy.GetAnalyzedImage()->GetNumLevels();

            std::vector<MapPointAssociations<MapPointTrackingProxy>::Association> extraAssociations;

            for (const auto& mapPoint : mapPoints)
            {
                if (keyFrameproxy.HasMapPoint(mapPoint.GetId()))
                {
                    continue;
                }

                bool predicted = false;
                KeypointDescriptorIndex keypointDescriptorIndex;

                if (TrackLocalMap::ProjectMapPointIntoCurrentFrame(keyFrameproxy, viewMatrix, cameraCalibrationMatrix, mapPoint, framePosition,
                    frameForward, settings.TrackLocalMapSettings.MinDegreesBetweenCurrentViewAndMapPointView, settings.LoopClosureSettings.MatchSearchRadius,
                    imageBorder,
                    pyramidScale,
                    numLevels,
                    settings.LoopClosureSettings.CheapLoopClosureMatchingSettings,
                    unassociatedMask, memory, predicted, keypointDescriptorIndex))
                {
                    extraAssociations.emplace_back(mapPoint.As<MapPointTrackingProxy>(), keypointDescriptorIndex);
                    unassociatedMask[keypointDescriptorIndex] = false;
                }
            }

            return extraAssociations;
        }
    }

    struct MappingWorker::Impl
    {
        // Max size 72 is required for x64 builds
        mira::background_dispatcher<72> Dispatcher;
        mira::determinator& Determinator;
        MageContext& Context;
        const MageSlamSettings& Settings;
        const PerCameraSettings& CameraSettings;

        mira::task<Id<Keyframe>> Previous = mira::task_from_result(Id<Keyframe>::id());
        int FramesProcessed = 0;
        unsigned int CosVisThreashold{};
        boost::optional<float> CurrentLambda{};
        std::atomic<bool> MappingWorkAvailable{ false };

        MappingWorker::Impl(
            mira::determinator& determinator,
            MageContext& context,
            const MageSlamSettings& settings,
            const PerCameraSettings& cameraSettings)
            :   Dispatcher{},
                Determinator{ determinator },
                Context{ context },
                Settings{ settings },
                CameraSettings{ cameraSettings }
        {
            Dispatcher.queue([]() { mage::platform::set_thread_name("Mage MappingWorker Thread"); });
        }

        struct
        {
            std::shared_ptr<const KeyframeBuilder> KeyFrameBuilder;
            mira::unique_vector<Id<MapPoint>> MapPointsThatFailedScoring;
            std::vector<MapPointAssociations<MapPointTrackingProxy>::Association> LoopClosureAssociations;

            AdjustableData Data;
            std::unique_ptr<BundleAdjustTask> BA;

            Id<Keyframe> Ki() const
            {
                return KeyFrameBuilder->GetId();
            }

            void Clear()
            {
                BA.reset();
                Data.Clear();

                KeyFrameBuilder.reset();
                MapPointsThatFailedScoring.clear();
                LoopClosureAssociations.clear();
            }

        } PerIteration{};
    };

    MappingWorker::MappingWorker(
        mira::determinator& determinator,
        MageContext& context,
        const MageSlamSettings& settings,
        const PerCameraSettings& cameraSettings)
        : BaseWorker{ 100 * 1024, 1000 * 1024 },
        m_impl{ std::make_unique<Impl>(determinator, context, settings, cameraSettings) }
    {
        m_impl->CosVisThreashold = settings.CovisibilitySettings.CovisMinThreshold;
    }

    void MappingWorker::SetMappingWorkAvailable(bool available)
    {
        m_impl->MappingWorkAvailable = available;
    }

    mira::task<Id<Keyframe>> MappingWorker::MappingTask(const PoseRefined& poseRefined)
    {
        assert(poseRefined.IsNewKeyframe);

        m_impl->Previous = m_impl->Previous.then(m_impl->Dispatcher, Cancellation(),
            [this, poseRefined](const mira::expected<Id<Keyframe>>&)
        {
            m_impl->FramesProcessed++;

            m_impl->PerIteration.KeyFrameBuilder = std::move(poseRefined.Frame);
            m_impl->PerIteration.MapPointsThatFailedScoring = std::move(poseRefined.MapPointsThatFailedScoring);

            // Cheap Loop Closure
            if (m_impl->Settings.MappingSettings.UseCheapLoopClosure)
            {
                m_impl->PerIteration.LoopClosureAssociations =
                    CheapLoopClosure(m_impl->Context.Map, m_impl->Settings, *m_impl->PerIteration.KeyFrameBuilder, m_impl->FramesProcessed, MemoryPool().create());
            }

            return m_impl->Context.StateMachine.on(KeyframeInsertionAndMapPointCullingState, m_impl->Dispatcher, Cancellation(),
                [this]()
            {
                auto memory = MemoryPool().create();

                auto& kf = m_impl->PerIteration.KeyFrameBuilder;

                // KEYFRAME INSERTION
                m_impl->Context.Map.InsertKeyframe(kf, m_impl->PerIteration.LoopClosureAssociations, memory);
                m_impl->Context.History.AddKeyframeToTrackingHistory(kf->GetId(), kf->GetPose());

                // update any connected keyframes by adding associations from cheap loop closure
                m_impl->Context.Map.TryConnectMapPoints(m_impl->PerIteration.LoopClosureAssociations, kf->GetId(), m_impl->Settings.TrackLocalMapSettings,
                    m_impl->Settings.TrackLocalMapSettings.OrbMatcherSettings, m_impl->Settings.LoopClosureSettings.MatchSearchRadius,
                    memory);

                // RECENT MAP POINT CULLING
                m_impl->Context.Map.CullRecentMapPoints(kf->GetId(), m_impl->PerIteration.MapPointsThatFailedScoring, memory);

            });
        }).then(m_impl->Dispatcher, Cancellation(), [this]
        {
            // add the keyframe to the BOW instance
            auto& kf = m_impl->PerIteration.KeyFrameBuilder;
            m_impl->Context.BagOfWords.AddImage(kf->GetId(), *kf->GetAnalyzedImage());

        }).then(m_impl->Dispatcher, Cancellation(), [this, &poseRefined]()
        {
            // NEW POINT CREATION
            SCOPE_TIMER(Scheduling::MapPointCreation);

            auto memory = MemoryPool().reuse();

            auto Ki = m_impl->PerIteration.Ki();

            //get copy of connected keyframes and store in mapping frames for use by mapping thread
            std::vector<MappingKeyframe> Kcs;
            m_impl->Context.Map.GetCovisibilityConnectedKeyframes(Ki, memory, Kcs);

            std::vector<MapPointKeyframeAssociations> newMapPoints;
            NewMapPointsCreation(
                m_impl->Context.BagOfWords,
                *m_impl->Context.Map.GetKeyFrameProxy(Ki),
                m_impl->CameraSettings,
                m_impl->Settings.MappingSettings.NewMapPointsCreationSettings,
                m_impl->Context.Map.GetMapScale(),
                memory,
                Kcs,
                newMapPoints);

            FIRE_OBJECT_TRACE("Mappoints.Created",
                              nullptr, make_frame_data_point(*poseRefined.Frame, (float)newMapPoints.size()));

            return m_impl->Context.StateMachine.on(MapPointCreationState, m_impl->Dispatcher, Cancellation(),
                [this, Ki, newMapPoints = std::move(newMapPoints)]()
            {
                auto memory = MemoryPool().reuse();

                m_impl->Context.Map.CreateMapPoints(m_impl->PerIteration.Ki(), newMapPoints, memory);
                DETERMINISTIC_CHECK(m_impl->Determinator, newMapPoints.size());
            });
        }).then(m_impl->Dispatcher, Cancellation(), [this]()
        {
            auto memory = MemoryPool().reuse();

            auto Ki = m_impl->PerIteration.Ki();

            auto& baData = m_impl->PerIteration.Data;

            //LOCAL BUNDLE ADJUSTMENT
            m_impl->CosVisThreashold = m_impl->Context.Map.GetMapPointsAndDistantKeyframes(
                Ki,
                m_impl->CosVisThreashold,
                memory,
                baData.MapPoints,
                baData.Keyframes,
                baData.MapPointAssociations,
                baData.ExternallyTetheredKeyframes
            );

            DETERMINISTIC_CHECK(m_impl->Determinator, m_impl->CosVisThreashold);
            DETERMINISTIC_CHECK(m_impl->Determinator, baData.MapPoints);
            DETERMINISTIC_CHECK(m_impl->Determinator, baData.MapPointAssociations);
            DETERMINISTIC_CHECK(m_impl->Determinator, baData.Keyframes);

            // if the size of the localmap is small, then we can do more iterations without
            // overflowing our time budgets.  This is intended to increase accuracy in
            // sparse regions and during startup
            size_t connectivityRatio = m_impl->Settings.CovisibilitySettings.UpperConnectionsForBA / baData.MapPointAssociations.size();

            size_t numStepsPerRun = m_impl->Settings.BundleAdjustSettings.NumStepsPerRun;
            float huberWidth = m_impl->Settings.BundleAdjustSettings.HuberWidth;
            if (connectivityRatio > 0)
            {
                numStepsPerRun *= static_cast<size_t>(connectivityRatio * m_impl->Settings.BundleAdjustSettings.LowConnectivityIterationsScale);
                huberWidth *= powf(m_impl->Settings.BundleAdjustSettings.HuberWidthScale, static_cast<float>(connectivityRatio));
            }

            m_impl->PerIteration.BA = std::make_unique<BundleAdjustTask>(
                m_impl->PerIteration.Data, m_impl->Determinator,
                m_impl->Settings.BundleAdjustSettings.MaxOutlierError,
                m_impl->Settings.BundleAdjustSettings.MaxOutlierErrorScaleFactor,
                false, // fix map points
                numStepsPerRun);

            if (m_impl->Settings.MappingSettings.PersistLambda && m_impl->CurrentLambda)
            {
                // Persisting the lambda across iterations of bundle adjustment is a hack to prevent the
                // learnings from being discarded between every bundle adjust.  This pertains to a behavior
                // of the Levenberg-Marquardt solver, which is supposed to initialize lambda arbitrarily and
                // optimize it to a reasonable value over several iterations.  However, because we have so
                // little time to iterate before we need to rebuild the bundle adjuster, lambda is often
                // unable to achieve a reasonable value within the span of a single local bundle adjust, thus
                // crippling the bundle adjuster's ability to optimize.  Because we expect each local bundle
                // adjust to be a similar optimization problem, we can compensate for the above problem by
                // seeding each bundle adjustment with the final lambda of the prior bundle adjustment,
                // thereby allowing the information described by the lambda to be preserved and refined over
                // time.
                m_impl->PerIteration.BA->SetCurrentLambda(m_impl->CurrentLambda.value());
            }

            return IterateBA(huberWidth);
        }).then(mira::inline_scheduler, Cancellation(), [this]
        {
            return m_impl->Context.StateMachine.on(KeyframeCullingState, m_impl->Dispatcher, Cancellation(), [this]
            {
                m_impl->CurrentLambda = std::max(m_impl->PerIteration.BA->GetCurrentLambda(), m_impl->Settings.MappingSettings.MinLambda.value);

                auto memory = MemoryPool().reuse();
                auto Ki = m_impl->PerIteration.Ki();

                m_impl->Context.History.ConnectAdjustedKeyframeToNewlyEstimatedPoses(Ki, m_impl->Context.Map.GetKeyFrameProxy(Ki)->GetPose());

                // create a storage container so that culling can tell us what it just did
                std::vector<std::pair<const Id<Keyframe>, const std::vector<Id<Keyframe>>>> culledKeyframes;

                m_impl->Context.Map.CullLocalKeyframes(Ki, memory, &culledKeyframes);

                // tell the history about the culled keyframes
                for (const std::pair<const Id<Keyframe>, const std::vector<Id<Keyframe>>>& culledPair : culledKeyframes)
                {
                    m_impl->Context.History.RemoveKeyframeFromTrackingHistory(culledPair.first, culledPair.second);
                }
                DETERMINISTIC_CHECK(m_impl->Determinator, culledKeyframes.size());

                LogMessage<Tracing::TraceLevels::Verbose>((boost::wformat(L"MappingThread: Map Point Count: %d") % m_impl->Context.Map.GetMapPointsCount()).str());
                LogMessage<Tracing::TraceLevels::Verbose>((boost::wformat(L"MappingThread: Keyframe Count: %d") % m_impl->Context.Map.GetKeyframesCount()).str());

                return m_impl->PerIteration.Ki();
            });
        }).then(mira::inline_scheduler, mira::cancellation::none(), [this](const mira::expected<Id<Keyframe>>& result)
        {
            // clear all our per iteration data before the next even on error.
            m_impl->PerIteration.Clear();

            m_impl->Context.TickImageFactories(MageContext::ThreadType::MappingThread);

            return result;
        });

        return Pending() += m_impl->Previous;
    }

    mira::task<void> MappingWorker::IterateBA(float huberWidth)
    {
        m_impl->PerIteration.BA->IterateBundleAdjust(huberWidth);

        auto& state = m_impl->PerIteration.BA->GetIterations() == 1 ? BundleAdjustFirstWriteToMapState : BundleAdjustNthWriteToMapState;

        return m_impl->Context.StateMachine.on(state, m_impl->Dispatcher, Cancellation(),
            [this](bool& shouldKeepIterating)
            {
                auto memory = MemoryPool().reuse();

                auto& ba = *m_impl->PerIteration.BA;

                auto currentOutliers = ba.GetCurrentIterationOutliers();

                SCOPE_TIMER(AdjustPosesAndPositions);
                m_impl->Context.Map.AdjustPosesAndMapPoints(ba.GetData(), currentOutliers, memory);
                m_impl->Context.History.AdjustPoses(ba.GetData().Keyframes);

                DETERMINISTIC_CHECK(m_impl->Determinator, currentOutliers.size());
                DETERMINISTIC_CHECK(m_impl->Determinator, ba.GetData().MapPoints.size());
                DETERMINISTIC_CHECK(m_impl->Determinator, ba.GetData().Keyframes.size());
                DETERMINISTIC_CHECK(m_impl->Determinator, ba.GetData().MapPointAssociations.size());

                // we should keep iterating if there's nothing
                // waiting for us in the keyframe queue, this is
                // the early out logic in the paper
                shouldKeepIterating =
                    !m_impl->MappingWorkAvailable &&
                    ba.GetTotalSteps() < m_impl->Settings.BundleAdjustSettings.NumSteps &&
                    (ba.GetResidualError() > m_impl->Settings.BundleAdjustSettings.MinMeanSquareError || ba.GetTotalSteps() < m_impl->Settings.BundleAdjustSettings.MinSteps);
                return shouldKeepIterating;

            }).then(m_impl->Dispatcher, Cancellation(),
                [this, huberWidth](bool keepIterating)
                {
                    if (keepIterating)
                        return IterateBA(huberWidth);
                    else
                        return mira::task_from_result();
                });
    }

    MappingWorker::~MappingWorker() = default;
}
