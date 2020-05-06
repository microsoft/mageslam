// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "TrackingEvents.h"

#include "BaseWorker.h"

#include <arcana/threading/task.h>

namespace mage
{
    struct LoopClosureTrackingUpdate;
    struct MageSlamSettings;
    struct MageContext;
    class ThreadSafePoseHistory;

    class TrackLocalMapWorker : public BaseWorker
    {
    public:
        TrackLocalMapWorker(
            gsl::span<KeyframeProxy> keyframes,
            TrackingMediator& mediator,
            mira::determinator& determinator,
            MageContext& context,
            const MageSlamSettings& settings);

        void SetMappingIdle();

        void ConsumeUpdateFromLoopClosure(const LoopClosureTrackingUpdate&);

        const TrackingFrameHistory& GetHistory() const;

        mira::task<PoseRefined> RefinePose(const std::shared_ptr<const PoseEstimated>& estimated);

        ~TrackLocalMapWorker();

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
