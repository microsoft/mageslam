// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "LoopClosureWorker.h"

#include "Map/ThreadSafeMap.h"
#include "Tracking/PoseEstimator.h"
#include "Tracking/TrackLocalMap.h"
#include "Utils/Logging.h"

#include "Platform/Platform.h"

#include <arcana/threading/dispatcher.h>

namespace
{
    struct LoopClosureRelocalizationClusters
    {
        LoopClosureRelocalizationClusters(std::unique_ptr<mage::KeyframeProxy> keyframe)
            : Keyframe{ std::move(keyframe) }
        {}

        std::unique_ptr<mage::KeyframeProxy> Keyframe;
        std::vector<std::shared_ptr<mage::KeyframeProxy>> Relocalizations;
        std::vector<std::vector<mage::KeyframeProxy>> Clusters;
    };

    struct LoopClosureConfirmedLoopInformation
    {
        mage::KeyframeProxy OriginalKeyframe;
        mage::KeyframeProxy RelocalizedKeyframe;
        std::vector<mage::KeyframeProxy> Cluster;
        cv::Matx44f UnscaledSimilarityTransform;
        float SimilarityScale;

        // TODO: Optimize loop closure loop information structure, including minimizing copies
        LoopClosureConfirmedLoopInformation(
            const mage::KeyframeProxy& original,
            const mage::KeyframeProxy& relocal,
            std::vector<mage::KeyframeProxy> cluster,
            const cv::Matx44f& simTransform,
            float simScale)
            :   OriginalKeyframe{ original },
                RelocalizedKeyframe{ relocal },
                Cluster{ std::move(cluster) },
                UnscaledSimilarityTransform{ simTransform },
                SimilarityScale{ simScale }
        {}
    };
}

namespace mage
{
    struct LoopClosureWorker::Impl
    {
        // Max size 72 is required for x64 builds
        mira::background_dispatcher<72> Dispatcher;
        mira::determinator& Determinator;
        MageContext& Context;
        const MageSlamSettings& Settings;

        PoseEstimator PoseEstimator;
        TrackLocalMap TrackLocalMap;

        mira::task<void> Previous = mira::task_from_result();

        Impl(const MageSlamSettings& settings,
            MageContext& context)
            :   Dispatcher{},
                Determinator{ mira::determinator::create("LoopClosure") },
                Context{ context },
                Settings{ settings },
                PoseEstimator{ settings.PoseEstimationSettings, settings.RelocalizationSettings },
                TrackLocalMap{ settings, context.IMUCharacterization, Determinator }
        {
            Dispatcher.queue([]() { mage::platform::set_thread_name("Mage LoopClosure Thread"); });
        }
    };

    LoopClosureWorker::LoopClosureWorker(
        MageContext& context,
        const MageSlamSettings& settings)
        : BaseWorker{ 100 * 1024, 1000 * 1024 },
        m_impl{ std::make_unique<Impl>(settings, context) }
    {
    }

    LoopClosureWorker::~LoopClosureWorker() = default;

    mira::task<void> LoopClosureWorker::AttemptLoopClosure(const Id<Keyframe> kfId)
    {
        m_impl->Previous = m_impl->Previous.then(m_impl->Dispatcher, Cancellation(), [this, kfId]
        {
            thread_memory memory = MemoryPool().create();
            return DetectLoop(kfId, memory)
                .then(m_impl->Dispatcher, Cancellation(), [this, memory](const std::shared_ptr<LoopClosureConfirmedLoopInformation>& detection)
            {
                return CloseLoop(detection, memory);
            });
        }).then(m_impl->Dispatcher, Cancellation(), [this](const mira::expected<void>&)
        {
            m_impl->Context.TickImageFactories(MageContext::ThreadType::LoopClosureThread);
        });

        return Pending() += m_impl->Previous;
    }

    mira::task<std::shared_ptr<LoopClosureConfirmedLoopInformation>> LoopClosureWorker::DetectLoop(const Id<Keyframe> id, thread_memory memory)
    {
        return m_impl->Context.StateMachine.on(LoopDetectionState, m_impl->Dispatcher, Cancellation(), [this, id, memory](bool& loopDetected)
        {
            loopDetected = false;
            if (!m_impl->Settings.LoopClosureSettings.EnableLoopClosure)
                return mira::task_from_error<std::shared_ptr<LoopClosureConfirmedLoopInformation>>(mira::errc::failed);

            return mira::make_task(m_impl->Dispatcher, Cancellation(), [this, id, memory]() -> mira::expected<std::shared_ptr<LoopClosureRelocalizationClusters>>
            {
                // Finds keyframes that are visually similar to the loop keyframe but not already covisible
                // with the loop keyframe, then clusters them by mutual covisibility.  Each found cluster
                // will serve as a possible destination for a loop closure, if it survives other tests.
                // Blocks the map.

                std::shared_ptr<LoopClosureRelocalizationClusters> relocs = std::make_shared<LoopClosureRelocalizationClusters>(std::move(m_impl->Context.Map.GetKeyFrameProxy(id)));
                m_impl->Context.Map.FindNonCovisibleSimilarKeyframeClusters(*relocs->Keyframe, memory, relocs->Clusters);

                if (relocs->Keyframe == nullptr)
                    return mira::errc::failed;

                return relocs;
            }).then(m_impl->Dispatcher, Cancellation(), [this, memory](const std::shared_ptr<LoopClosureRelocalizationClusters>& relocs)
            {
                // Attempts pose estimation based on each of the relocalization clusters.  This sets up the
                // similarity transform and serves as an initial winnowing for the possible loop closures.
                // Does not lock the map.

                FindLoopCandidates(*relocs, memory);
                return relocs;
            }).then(m_impl->Dispatcher, Cancellation(), [this, &loopDetected, memory](const std::shared_ptr<LoopClosureRelocalizationClusters>& relocs)
            {
                // Collects the keyframe sets that will be used to test the relocalization clusters.  Locks
                // the map.

                auto connectedKeyframeSets = std::make_shared<std::vector<loop::vector<KeyframeReprojection>>>();
                GetConnectedKeyframeSets(*relocs, memory, *connectedKeyframeSets);

                return mira::make_task(m_impl->Dispatcher, Cancellation(), [this, &loopDetected, relocs, connectedKeyframeSets, memory]()
                    -> mira::expected<std::shared_ptr<LoopClosureConfirmedLoopInformation>>
                {
                    // Evaluates all possible loop closures and, if any are determined to be real, chooses one
                    // of those and computes the similarity transform.  Does not lock the map.

                    std::shared_ptr<LoopClosureConfirmedLoopInformation> li{};
                    if (!SelectLoopClosureCandidateCluster(*relocs, *connectedKeyframeSets, memory, li))
                        return mira::errc::failed;

                    loopDetected = true;
                    return li;
                });
            });
        });
    }

    mira::task<void> LoopClosureWorker::CloseLoop(const std::shared_ptr<LoopClosureConfirmedLoopInformation>& detection, thread_memory memory)
    {
        std::shared_ptr<AdjustableData> data = std::make_shared<AdjustableData>();

        return m_impl->Context.StateMachine.on(StartLoopClosureState, m_impl->Dispatcher, Cancellation(),
            [this, detection, memory, data](LoopClosureTrackingUpdate& trackingUpdate)
        {
            assert(trackingUpdate.MapPointMerges->empty());

            // Collects the covisible set of the loop closure keyframe, which will be combined with the
            // confirmed loop information and used to merge map points and reposition the covisible set.
            // Locks the map.
            std::shared_ptr<std::vector<MappingKeyframe>> covisibleKeyframes = std::make_shared<std::vector<MappingKeyframe>>();
            m_impl->Context.Map.GetCovisibilityConnectedKeyframes(detection->OriginalKeyframe.GetId(), memory, *covisibleKeyframes);

            // Makes new keyframe proxies that represent the moved loop covisible set.  Does not lock
            // the map.
            std::shared_ptr<std::vector<KeyframeProxy>> moved = std::make_shared<std::vector<KeyframeProxy>>();
            CloseDetectedLoop(*detection, *covisibleKeyframes, memory, *moved);

            // Consumes the new keyframe proxies, which triggers map point merge and the
            // repositioning of the loop keyframe covisible set.  Then runs a global bundle
            // adjust to fix the misalignments introduced by the loop closure.  Locks the map
            // in a big way.
            m_impl->Context.Map.UpdateKeyframesFromProxies(*moved, m_impl->Settings.LoopClosureSettings.MapMergeMatchingSettings, *trackingUpdate.MapPointMerges, memory);
            m_impl->Context.Map.BuildGlobalBundleAdjustData(*data);
            m_impl->Context.History.AdjustPoses(data->Keyframes);
        }).then(m_impl->Dispatcher, Cancellation(), [this, memory, data]
        {
            std::shared_ptr<BundleAdjustTask> ba = std::make_shared<BundleAdjustTask>(
                *data,
                m_impl->Determinator,
                m_impl->Settings.LoopClosureSettings.BundleAdjustSettings.MaxOutlierError,
                m_impl->Settings.LoopClosureSettings.BundleAdjustSettings.MaxOutlierErrorScaleFactor,
                false,
                m_impl->Settings.LoopClosureSettings.BundleAdjustSettings.NumStepsPerRun);

            ba->IterateBundleAdjust(m_impl->Settings.LoopClosureSettings.BundleAdjustSettings.HuberWidth);

            return m_impl->Context.StateMachine.on(EndLoopClosureState, m_impl->Dispatcher, Cancellation(), [this, memory, data, ba]
            {
                m_impl->Context.Map.AdjustPosesAndMapPoints(*data, ba->GetOutliers(), memory);
                m_impl->Context.History.AdjustPoses(data->Keyframes);
            });
        });
    }

    void LoopClosureWorker::FindLoopCandidates(LoopClosureRelocalizationClusters& relocs, thread_memory memory)
    {
        // This approach essentially attempts to relocalize individually on each cluster of adequate size individually,
        // which ensures that every cluster is given a "fair shake."  There is another possible approach: one or more of
        // the frames from each sufficient cluster are added to a single superset, it is left to the reloc code to find the
        // most correct relocalization.  This does isolate the responsibility for finding the correct position into the 
        // relocalization code; the trick is that you need to know which cluster reloc actually succeeded from later on in
        // order to run TrackLocalMap on the local covisible set.  Note that these approaches are equivalent when there is
        // only one sufficient cluster, which is common.

        const proxy::Intrinsics intrinsics{
            {
                ArrayFromMat(relocs.Keyframe->GetAnalyzedImage()->GetUndistortedCalibration().GetLinearIntrinsics()),
                relocs.Keyframe->GetAnalyzedImage()->GetWidth(),
                relocs.Keyframe->GetAnalyzedImage()->GetHeight()
            } };

        for (auto& cluster : relocs.Clusters)
        {
            std::shared_ptr<KeyframeProxy> estimate = std::make_shared<KeyframeProxy>(
                relocs.Keyframe->GetId(),
                proxy::Image{ relocs.Keyframe->GetAnalyzedImage() },
                proxy::Pose{},
                intrinsics,
                proxy::Associations<MapPointTrackingProxy>{ relocs.Keyframe->GetAnalyzedImage() },
                false);

            if (cluster.size() >= m_impl->Settings.LoopClosureSettings.MinClusterSize &&
                m_impl->PoseEstimator.TryEstimatePoseFromCandidates(m_impl->Context.BagOfWords, cluster, m_impl->Determinator, memory, *estimate))
            {
                relocs.Relocalizations.push_back(estimate);
            }
            else
            {
                relocs.Relocalizations.emplace_back();
            }
        }
    }

    void LoopClosureWorker::GetConnectedKeyframeSets(const LoopClosureRelocalizationClusters& relocs, thread_memory memory, std::vector<loop::vector<KeyframeReprojection>>& connectedKeyframeSets)
    {
        std::vector<MapPointTrackingProxy> mapPoints;
        for (size_t idx = 0; idx < relocs.Relocalizations.size(); idx++)
        {
            connectedKeyframeSets.push_back(memory.loop_vector<KeyframeReprojection>());

            if (relocs.Relocalizations[idx] != nullptr)
            {
                mapPoints.clear();
                relocs.Relocalizations[idx]->GetMapPoints(mapPoints);

                m_impl->Context.Map.GetConnectedMapPoints(mapPoints, memory, connectedKeyframeSets[idx]);
            }
        }
    }

    bool LoopClosureWorker::SelectLoopClosureCandidateCluster(
        LoopClosureRelocalizationClusters& relocs,
        const std::vector<loop::vector<KeyframeReprojection>>& connectedKeyframeSets,
        thread_memory memory,
        std::shared_ptr<LoopClosureConfirmedLoopInformation>& selectedCluster)
    {
        auto currMask = relocs.Keyframe->GetAssociatedKeypointMask();

        for (size_t clusterIdx = 0; clusterIdx < relocs.Clusters.size(); clusterIdx++)
        {
            // We only care about potential loop closures where pose estimation succeeded.
            if (relocs.Relocalizations[clusterIdx] != nullptr)
            {
                auto& destinationKf = relocs.Relocalizations[clusterIdx];

                std::vector<MapPointTrackingProxy> mapPoints;
                destinationKf->GetMapPoints(mapPoints);

                const auto& connectedKeyframes = connectedKeyframeSets[clusterIdx];

                // We only care about potential loop closures that can survive a TrackLocalMap.
                if (m_impl->TrackLocalMap.RunTrackLocalMap(connectedKeyframes, {}, memory, false, true, *destinationKf))
                {
                    // 6DOF similarity transform for this newly-discovered position.
                    auto simT = To4x4(destinationKf->GetPose().GetRelativeViewMatrix(relocs.Keyframe->GetPose()));

                    // Final degree of freedom, estimate the scale.
                    auto relocMask = destinationKf->GetAssociatedKeypointMask();
                    assert(currMask.size() == relocMask.size());

                    // Recover the scale ratio by comparing the distances of keypoints seen by both frames.
                    float currDepth = 0.f;
                    float relocDepth = 0.f;
                    for (size_t idx = 0; idx < currMask.size(); idx++)
                    {
                        if (currMask[idx] && relocMask[idx])
                        {
                            cv::Vec3f vec = relocs.Keyframe->GetPose().GetWorldSpacePosition() - relocs.Keyframe->GetAssociatedMapPoint(idx)->GetPosition();
                            currDepth += cv::norm(vec);

                            vec = destinationKf->GetPose().GetWorldSpacePosition() - destinationKf->GetAssociatedMapPoint(idx)->GetPosition();
                            relocDepth += cv::norm(vec);
                        }
                    }

                    // We only care about potential loop closures that saw at least SOME of the same points, thus allowing us to estimate scale.
                    if (currDepth != 0.f && relocDepth != 0.f)
                    {
                        // We arbitrarily pick the first viable candidate, then close the loop there.  We could also 
                        // do a comparison among candidates (in the rare event when there's more than one), or we 
                        // could simply abandon multi-candidate relocalizations as potentially ambiguous.
                        selectedCluster = std::make_shared<LoopClosureConfirmedLoopInformation>(
                            *relocs.Keyframe,
                            *relocs.Relocalizations[clusterIdx],
                            relocs.Clusters[clusterIdx],
                            simT,
                            relocDepth / currDepth);

                        return true;
                    }
                }
            }
        }

        return false;
    }

    void LoopClosureWorker::CloseDetectedLoop(
        const LoopClosureConfirmedLoopInformation& li,
        const std::vector<MappingKeyframe>& covisibleKeyframes,
        thread_memory memory,
        std::vector<KeyframeProxy>& moved)
    {
        SCOPE_TIMER(LoopCloser::CloseDetectedLoop);

        std::vector<const KeyframeProxy*> clusterPointers;
        transform(li.Cluster.begin(), li.Cluster.end(), std::back_inserter(clusterPointers), [](auto& kf) { return &kf; });

        // Create the new representations of the local keyframes at the correct location.
        for (const auto& covisKf : covisibleKeyframes)
        {
            moved.emplace_back(
                covisKf.GetId(),
                proxy::Image{ covisKf.GetAnalyzedImage() },
                proxy::Pose{ covisKf.GetPose().GetInverseViewMatrix() * li.UnscaledSimilarityTransform },
                proxy::Intrinsics{ {
                        ArrayFromMat(covisKf.GetAnalyzedImage()->GetUndistortedCalibration().GetLinearIntrinsics()),
                        covisKf.GetAnalyzedImage()->GetUndistortedCalibration().GetCalibrationWidth(),
                        covisKf.GetAnalyzedImage()->GetUndistortedCalibration().GetCalibrationHeight()
                    } },
                proxy::Associations<MapPointTrackingProxy>{ covisKf.GetAnalyzedImage() },
                false
            );
            m_impl->PoseEstimator.TryEstimatePoseFromKeyframe(clusterPointers, m_impl->Settings.LoopClosureSettings.MinFeatureMatches, memory, moved.back());
        }

        moved.emplace_back(
            li.OriginalKeyframe.GetId(),
            proxy::Image{ li.RelocalizedKeyframe.GetAnalyzedImage() },
            proxy::Pose{ li.RelocalizedKeyframe.GetPose() },
            proxy::Intrinsics{ {
                    ArrayFromMat(li.RelocalizedKeyframe.GetAnalyzedImage()->GetUndistortedCalibration().GetLinearIntrinsics()),
                    li.RelocalizedKeyframe.GetAnalyzedImage()->GetUndistortedCalibration().GetCalibrationWidth(),
                    li.RelocalizedKeyframe.GetAnalyzedImage()->GetUndistortedCalibration().GetCalibrationHeight()
                } },
            proxy::Associations<MapPointTrackingProxy>{ li.RelocalizedKeyframe.GetAnalyzedImage() },
            false
        );
    }
}
