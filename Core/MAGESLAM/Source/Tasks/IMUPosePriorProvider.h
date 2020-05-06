// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseWorker.h"

#include "Fuser/Interfaces.h"
#include "Tracking/IPosePriorProvider.h"

#include <arcana/mixin_ptr.h>

namespace mage
{
    class IMUPosePriorProvider : public BaseWorker, public IPosePriorProvider, public IIMUReceiver
    {
    public:
        using ImuMixin = mira::mixin_ptr<IPredictor, IPoseEstimator, IVisualPoseReceiver, IIMUReceiver>;

        IMUPosePriorProvider(ImuMixin imu);
        ~IMUPosePriorProvider();

        mira::task<Pose> GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& timestamp) override;
        void OnTrackingLost() override;

        void AddSample(const mage::SensorSample& sample) override;
    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
