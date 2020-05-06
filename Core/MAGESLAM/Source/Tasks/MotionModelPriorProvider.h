// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseWorker.h"
#include "Tracking/IPosePriorProvider.h"

namespace mage
{
    class MotionModelPriorProvider : public BaseWorker, public IPosePriorProvider
    {
    public:
        MotionModelPriorProvider();

        mira::task<Pose> GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& nextPoseTime) override;
        void OnTrackingLost() override {};
    private:
    };
}
