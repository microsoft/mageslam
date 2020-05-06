// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseWorker.h"

#include "MageSlam.h"

#include <arcana/scheduling/state_machine.h>
#include <gsl/gsl>
#include <memory>

namespace mage
{
    struct MageContext;
    struct MageSlamSettings;
    struct FrameData;

    class Fuser;

    class Runtime
    {
    public:
        Runtime(const MageSlamSettings& settings, MageContext& context, Fuser& fuser, mira::state_machine_driver& driver);
        ~Runtime();

        void Run(gsl::span<const MAGESlam::CameraConfiguration> cameras);

        void TrackMono(std::shared_ptr<FrameData> frame);
        void TrackStereo(std::shared_ptr<FrameData> one, std::shared_ptr<FrameData> two);

        void AddSample(const mage::SensorSample& sample);
    private:
        struct Impl;
        const std::unique_ptr<Impl> m_impl;
    };
}
