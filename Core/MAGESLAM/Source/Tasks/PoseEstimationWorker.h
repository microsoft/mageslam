// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "ImageData.h"
#include "TrackingEvents.h"

#include "BaseWorker.h"

namespace mage
{
    struct MageSlamSettings;
    struct MageContext;

    class PoseEstimationWorker : public BaseWorker
    {
    public:
        PoseEstimationWorker(
            TrackingMediator& mediator,
            mira::determinator& determinator,
            Fuser& fuser,
            MageContext& context,
            const MageSlamSettings& settings);

        mira::task<std::shared_ptr<const PoseEstimated>> EstimatePose(const FrameAnalyzed& frame, const TrackingFrameHistory& history, const boost::optional<Pose>& posePrior);

        ~PoseEstimationWorker();

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
