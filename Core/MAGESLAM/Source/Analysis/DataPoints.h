// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/analysis/data_point.h>

#include "Data/Data.h"

#include "Proxies/KeyframeProxy.h"

namespace mage
{
    struct frame_data_point : public mira::data_point<double>
    {
        FrameId frame;

        frame_data_point(FrameId frame, time_point time, double value)
            : mira::data_point<double>{time, value}
            , frame{frame}
        {}
    };

    inline frame_data_point make_frame_data_point(const typename Proxy<Keyframe, proxy::Image>::ViewT& kf, double value)
    {
        return { kf->GetAnalyzedImage()->GetFrameId(), kf->GetAnalyzedImage()->GetTimeStamp(), value };
    }

    inline frame_data_point make_frame_data_point(FrameId frame, const typename frame_data_point::time_point& time, double value)
    {
        return { frame, time, value };
    }
}
