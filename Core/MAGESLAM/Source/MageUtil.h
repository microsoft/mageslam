// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <Data/Data.h>
#include <memory>

namespace mage
{
    // calculates the overlap of the source frame in the target frame
    // input:
    //  targetToSourceMat: matrix that moves from target frame of reference to source frame of reference (extrinsics)
    //  targetModel: camera model for target frame (expects undistorted images, so this is pinhole model only)
    //  sourceModel: camera model for source frame (expects undistorted images, so this is pinhole model only)
    //  depthMeters: the depth to calculate the overlap for (meters)
    // output:
    //  rect: an offset (x,y) and width, height in the target frame showing what is visible in the source frame
    mage::Rect CalculateOverlapCropSourceInTarget(const mage::Matrix& targetToSourceMat, const std::shared_ptr<const mage::calibration::CameraModel>& targetModel,
        const std::shared_ptr<const mage::calibration::CameraModel>& sourceModel, float depthMeters);
}
