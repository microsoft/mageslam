// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MotionModelPriorProvider.h"

namespace mage
{

    MotionModelPriorProvider::MotionModelPriorProvider()
        : BaseWorker{0, 0}
    {}

    mira::task<Pose> MotionModelPriorProvider::GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& nextPoseTime)
    {
        auto pose = EstimateNextPoseFromHistory(history, nextPoseTime);
        return mira::task_from_result<Pose>(std::move(pose));
    }
}