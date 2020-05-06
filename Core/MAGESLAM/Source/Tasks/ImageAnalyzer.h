// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "ImageData.h"
#include "BaseWorker.h"

#include <arcana/timer.h>
#include <arcana/threading/task.h>

namespace mira
{
    class determinator;
}

namespace mage
{
    struct MageSlamSettings;
    struct MageContext;

    //
    // ImageAnalyzer consumes FrameReceived events, analyzes the frame data and fires the FrameAnalyzed event
    // with the result.
    //
    class ImageAnalyzer : public BaseWorker
    {
    public:
        ImageAnalyzer(
            mira::determinator& determinator,
            MageContext& context,
            gsl::span<const MAGESlam::CameraConfiguration> configurations,
            const MageSlamSettings& settings);

        mira::task<FrameAnalyzed> ProcessFrame(const std::shared_ptr<FrameData>& frame);
        mira::task<StereoFramesAnalyzed> ProcessFrames(const std::shared_ptr<FrameData>& one, const std::shared_ptr<FrameData>& two);

        ~ImageAnalyzer();

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
