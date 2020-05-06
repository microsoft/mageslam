// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "TrackingEvents.h"
#include "MageContext.h"

namespace mage
{
    class ThreadSafePoseHistory;

    //
    // FuserWorker consumes FrameAnalyzed events, and from that data tries to initialize the map.
    //
    class FuserWorker
    {
    public:
        FuserWorker(
            TrackingMediator& mediator,
            mira::determinator& determinator,
            Fuser& fuser,
            MageContext& context,
            const MageSlamSettings& settings);

        ~FuserWorker();

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
