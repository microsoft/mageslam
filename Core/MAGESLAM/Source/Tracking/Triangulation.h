// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>


namespace mage
{
    void TriangulatePointsViewSpaceSlow(const cv::Matx34f& frame1CameraProjection, const cv::Matx34f& frame2CameraProjection, const std::vector<cv::Point2f>& frame1Matched2dPoints,
        const std::vector<cv::Point2f>& frame2Matched2dPoints, std::vector<cv::Point3f>& triangulated3dPoints);

    cv::Point3f TriangulatePointWorldSpace(const cv::Matx33f& invCalibration1, const cv::Matx44f& worldMat1, const cv::Matx33f& invCalibration2, const cv::Matx44f& worldMat2, const cv::Point2f& p1, const cv::Point2f& p2);
}
