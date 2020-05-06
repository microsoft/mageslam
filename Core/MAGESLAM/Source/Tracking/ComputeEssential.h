// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <opencv2\core\core.hpp>

namespace mira
{
    cv::Mat FindEssentialMat(cv::InputArray points1, cv::InputArray points2, float focalX, float focalY, const cv::Point2f& pp, float minimizationEpsilon);
}
