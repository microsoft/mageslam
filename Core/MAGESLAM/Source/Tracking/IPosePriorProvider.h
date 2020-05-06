// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "PoseEstimator.h"

#include <arcana/threading/task.h>

namespace mage
{
    class IPosePriorProvider
    {
    public:
        virtual ~IPosePriorProvider() {}

        virtual mira::task<Pose> GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& nextPoseTime) = 0;

        virtual void OnTrackingLost() = 0;
    };
}