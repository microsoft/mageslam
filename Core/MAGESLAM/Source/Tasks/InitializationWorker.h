// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once


#include "BaseWorker.h"

#include "ImageData.h"
#include "TrackingEvents.h"

#include <arcana/threading/task.h>

namespace mage
{
    struct MageSlamSettings;
    struct MageContext;
    class ThreadSafePoseHistory;

    //
    // InitializationWorker consumes FrameAnalyzed events, and from that data tries to initialize the map.
    //
    class InitializationWorker : public BaseWorker
    {
    public:
        InitializationWorker(
            mira::dispatcher<72>& dispatcher,
            mira::determinator& determinator,
            MageContext& context,
            const MageSlamSettings& settings);

        mira::task<PoseRefined> Initialize(const FrameAnalyzed& frame);

        ~InitializationWorker();

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
