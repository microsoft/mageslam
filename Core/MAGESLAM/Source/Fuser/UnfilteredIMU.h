// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Interfaces.h"

#include "Device/IMUCharacterization.h"

namespace mage
{
    class UnfilteredIMU final :
        public IPredictor,
        public IIMUReceiver,
        public IVisualPoseReceiver,
        public IPoseEstimator
    {
    public:
        UnfilteredIMU(const device::IMUCharacterization& imuCharacterization);
        ~UnfilteredIMU();

        mage::Pose GetPose() const override;

        void AddSample(const mage::SensorSample& sample) override;

        void PredictUpTo(mage::SensorSample::Timestamp timestamp) override;

        void AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp) override;

    private:
        struct Impl;
        std::shared_ptr<Impl> m_impl;
    };
}
