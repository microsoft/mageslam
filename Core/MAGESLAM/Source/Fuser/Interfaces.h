// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data/Pose.h"
#include "SensorSample.h"

namespace mage
{
    class IIMUReceiver
    {
    public:
        virtual ~IIMUReceiver() = default;

        virtual void AddSample(const mage::SensorSample& sample) = 0;
    };

    class IVisualPoseReceiver
    {
    public:
        virtual ~IVisualPoseReceiver() = default;

        virtual void AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp) = 0;
        virtual void OnTrackingLost() {};
    };

    class IPredictor
    {
    public:
        virtual ~IPredictor() = default;

        virtual void PredictUpTo(mage::SensorSample::Timestamp timestamp) = 0;
    };

    class IPoseEstimator
    {
    public:
        virtual ~IPoseEstimator() = default;

        virtual mage::Pose GetPose() const = 0;
    };
}
