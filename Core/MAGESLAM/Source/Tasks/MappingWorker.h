// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseWorker.h"
#include "MageContext.h"
#include "TrackingEvents.h"

namespace mage
{
    class MappingWorker : public BaseWorker
    {
    public:
        MappingWorker(
            mira::determinator& determinator,
            MageContext& context,
            const MageSlamSettings& settings,
            const PerCameraSettings& cameraSettings);

        ~MappingWorker();

        void SetMappingWorkAvailable(bool available);

        mira::task<Id<Keyframe>> MappingTask(const PoseRefined&);

    private:
        mira::task<void> IterateBA(float huberWidth);

        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
