// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data/FrameData.h"

#include <arcana/messaging/mediator.h>
#include <memory>

namespace mage
{
    struct FrameAnalyzed
    {
        std::shared_ptr<FrameData> SourceFrame;
        std::shared_ptr<AnalyzedImage> Analyzed;
    };

    struct StereoFramesAnalyzed
    {
        FrameAnalyzed One;
        FrameAnalyzed Two;
    };
}
