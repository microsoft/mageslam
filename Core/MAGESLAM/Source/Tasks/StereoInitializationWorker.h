// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once


#include "BaseWorker.h"

#include "ImageData.h"
#include "TrackingEvents.h"

#include "MageSlam.h"

#include <arcana/threading/task.h>

namespace mage
{
    struct MageSlamSettings;
    struct MageContext;
    class ThreadSafePoseHistory;

    //
    // StereoInitializationWorker consumes StereoFramesAnalyzed events, and from that data tries to initialize the map.
    //
    class StereoInitializationWorker : public BaseWorker
    {
    public:
        StereoInitializationWorker(
            mira::dispatcher<72>& dispatcher,
            mira::determinator& determinator,
            gsl::span<const MAGESlam::CameraConfiguration> cameraConfigurations,
            MageContext& context,
            const MageSlamSettings& settings);

        mira::task<PoseRefined> Initialize(const StereoFramesAnalyzed& frame);

        ~StereoInitializationWorker();

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
