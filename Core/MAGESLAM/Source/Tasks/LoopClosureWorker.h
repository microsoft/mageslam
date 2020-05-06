// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseWorker.h"
#include "MageContext.h"

namespace
{
    struct LoopClosureRelocalizationClusters;
    struct LoopClosureConfirmedLoopInformation;
}

namespace mage
{
    struct LoopClosureTrackingUpdate
    {
        std::shared_ptr<std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>> MapPointMerges;

        LoopClosureTrackingUpdate()
            : MapPointMerges{ std::make_shared<std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>>() }
        {}
    };

    class LoopClosureWorker : public BaseWorker
    {
    public:
        LoopClosureWorker(
            MageContext& context,
            const MageSlamSettings& settings);

        ~LoopClosureWorker();

        mira::task<void> AttemptLoopClosure(const Id<Keyframe> kfId);

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;

        mira::task<std::shared_ptr<LoopClosureConfirmedLoopInformation>> DetectLoop(const Id<Keyframe>, thread_memory);
        mira::task<void> CloseLoop(const std::shared_ptr<LoopClosureConfirmedLoopInformation>&, thread_memory);

        void FindLoopCandidates(LoopClosureRelocalizationClusters&, thread_memory);
        void GetConnectedKeyframeSets(const LoopClosureRelocalizationClusters&, thread_memory, std::vector<loop::vector<KeyframeReprojection>>&);
        bool SelectLoopClosureCandidateCluster(LoopClosureRelocalizationClusters&, const std::vector<loop::vector<KeyframeReprojection>>&, thread_memory, std::shared_ptr<LoopClosureConfirmedLoopInformation>&);

        void CloseDetectedLoop(const LoopClosureConfirmedLoopInformation&, const std::vector<MappingKeyframe>&, thread_memory, std::vector<KeyframeProxy>&);
    };
}

