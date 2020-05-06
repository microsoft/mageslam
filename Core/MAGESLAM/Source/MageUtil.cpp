// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MageUtil.h"

#include "Utils/cv.h"
#include "Utils/Logging.h"
#include "Tracking/Reprojection.h"
#include "Device/CameraCalibration.h"

namespace mage
{
    mage::Rect CalculateOverlapCropSourceInTarget(const mage::Matrix& targetToSourceMat, const std::shared_ptr<const mage::calibration::CameraModel>& targetModel,
        const std::shared_ptr<const mage::calibration::CameraModel>& sourceModel, float depthMeters)
    {
        SCOPE_TIMER(MAGESlam::CalculateOverlapCropSourceInTarget);

        const CameraCalibration sourceCalibration(sourceModel);
        const CameraCalibration targetCalibration(targetModel);
        const cv::Matx44f targetToSource = mage::ToCVMat4x4(targetToSourceMat);

        assert(targetCalibration.GetDistortionType() == calibration::DistortionType::None && "expecting a null distortion, as this function wants images that are pre-undistorted");
        assert(sourceCalibration.GetDistortionType() == calibration::DistortionType::None && "expecting a null distortion, as this function wants images that are pre-undistorted");

        // create points in the corners of source frame
        const float maxSourceCol = (float)sourceCalibration.GetCalibrationWidth() - 1;
        const float maxSourceRow = (float)(sourceCalibration.GetCalibrationHeight() - 1);
        std::array<cv::Point2f, 4> ptsSource{ { { 0, 0 },{ maxSourceCol, 0 },{ 0,maxSourceRow },{ maxSourceCol, maxSourceRow } } };

        // project the 4 corners of the source into the target frame
        cv::Point2f minSourceInTargetPt{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        cv::Point2f maxSourceInTargetPt{ std::numeric_limits<float>::min(), std::numeric_limits<float>::min() };
        for (const auto& sourcePt : ptsSource)
        {
            cv::Point3f posWorld = mage::UnProject(sourceCalibration.GetInverseCameraMatrix(), targetToSource, { (int)sourcePt.x, (int)sourcePt.y }, depthMeters);
            mage::Projection projTargetFrame = mage::ProjectUndistorted(cv::Matx34f::eye(), targetCalibration.GetCameraMatrix(), posWorld);

            assert(projTargetFrame.Distance > 0 && "selected depth is behind the other camera");

            minSourceInTargetPt.x = std::min(minSourceInTargetPt.x, projTargetFrame.Point.x);
            minSourceInTargetPt.y = std::min(minSourceInTargetPt.y, projTargetFrame.Point.y);
            maxSourceInTargetPt.x = std::max(maxSourceInTargetPt.x, projTargetFrame.Point.x);
            maxSourceInTargetPt.y = std::max(maxSourceInTargetPt.y, projTargetFrame.Point.y);
        }

        assert(minSourceInTargetPt.x != std::numeric_limits<float>::max() && minSourceInTargetPt.y != std::numeric_limits<float>::max() && "no points in front of camera");
        assert(maxSourceInTargetPt.x != std::numeric_limits<float>::min() && maxSourceInTargetPt.y != std::numeric_limits<float>::min() && "no points in front of camera");

        float width = maxSourceInTargetPt.x - minSourceInTargetPt.x + 1;
        float height = maxSourceInTargetPt.y - minSourceInTargetPt.y + 1;
        assert(width >= 0 && "expecting a positive width for crop rect");
        assert(height >= 0 && "expecting a positive height for crop rect");

        return { (int)minSourceInTargetPt.x, (int)minSourceInTargetPt.y, (size_t)width, (size_t)height };
    }
}
