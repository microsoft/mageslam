// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "opencv2\core.hpp"

namespace mage
{
    class Pose;
    class CameraCalibration;

    cv::Matx33f ComputeEssentialMatrix(const Pose& fromFrame, const Pose& toFrame);
    cv::Matx33f ComputeFundamentalMatrix(const Pose& fromFrame, const CameraCalibration& fromFrameCalibration, const Pose& toFrame, const CameraCalibration& toFrameCalibration);

    float DistanceFromEpipolarLineSlow(const cv::Matx33f& fundamentalMat, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2);
    float DistanceFromEpipolarLine(const cv::Matx33f& fundamentalMat, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2);
    bool IsPointOnEpipolarLine(const cv::Matx33f& fundamentalMat, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2, const float errorThreshold);

    void DebugRenderEpipolarLine(const cv::Matx33f& f, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2, cv::Mat& frame2);
}
