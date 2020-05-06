// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Interfaces.h"

namespace mage
{
    class PoseInterpolator final :
        public IPredictor,
        public IVisualPoseReceiver,
        public IPoseEstimator
    {
    public:
        PoseInterpolator();
        ~PoseInterpolator();

        mage::Pose GetPose() const override;

        void PredictUpTo(mage::SensorSample::Timestamp timestamp) override;

        void AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp) override;

    private:
        struct Impl;
        std::shared_ptr<Impl> m_impl;
    };
}
