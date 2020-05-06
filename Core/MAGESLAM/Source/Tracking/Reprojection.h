// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>

#include <utility>
#include <gsl\gsl>
#include <vector>

namespace mage
{
    class CameraCalibration;

    struct Projection
    {
        cv::Point2f Point;
        float Distance;
    };

    void ProjectPoints(const gsl::span<const cv::Point3f>& points3D, const cv::Matx34f& cameraPose, const cv::Matx33f& calibrationMatrix, std::vector<Projection>& points2D);

    Projection ProjectUndistorted(const cv::Matx34f& viewMatrix, const cv::Matx33f& calibration, const cv::Point3f& world);
    Projection ProjectDistorted(const CameraCalibration& cameraCalibration, const cv::Matx34f& viewMatrix, const cv::Point3f& world);

    std::vector<cv::Point2f> ProjectDistorted(
        const mage::CameraCalibration& calibration,
        const cv::Matx34f& viewMatrix,
        gsl::span<const cv::Vec3f> points3D);
}
