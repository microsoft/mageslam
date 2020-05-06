// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data/Data.h"
#include <gsl/gsl>

// forward declare cv::KeyPoint
namespace cv
{
    class KeyPoint;
}

namespace mage
{
    struct InitializationData;
    struct FrameData;
    class AnalyzedImage;

    class Introspector
    {
    public:
        virtual ~Introspector() = default;

        virtual void Introspect(const InitializationData& /*data*/) {}

        virtual void IntrospectEstimatedPose(const mage::FrameId& /*frameId*/, const mage::Matrix& /*viewMatrix*/) {}

        virtual void IntrospectAnalyzedImage(const mage::FrameData& /*frame*/, const mage::AnalyzedImage& /* image */) {}
    };
}
