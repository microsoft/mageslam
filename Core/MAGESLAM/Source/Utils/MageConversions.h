// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once
#include "Data\Pose.h"
#include "Data\Data.h"

#include <gsl\gsl>

namespace mage
{
    /**
     * Converts the contents of a CVMat into the mage::Matrix data container by copying the values
     */
    Matrix ToMageMat(const cv::Matx34f& cvMat);
    Matrix ToMageMat(const cv::Matx44f& cvMat);

    Position ToMagePos(const cv::Point3f& cvPt);
    Direction ToMageDir(const cv::Point3f& cvDir);
    
    cv::Vec3f FromMageDir(const Direction& mageDir);

    Matrix ToMageMat(const std::array<float, 4 * 4>& f);

    Direction ToMageDir(const std::array<float, 3>& dir);

    mage::Pose MageMatrixToMagePose(const mage::Matrix& viewMatrix);

    void ConvertBGRToNV12(const cv::Mat& srcImg, gsl::span<uint8_t> destBuffer);

    bool IsIdentity(const Matrix& m);

    inline mage::Matrix CreateIdentityMageMatrix()
    {
        return{ 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f };
    };

    // Interpolates values between min and max to the range defined from a to b,
    // [a:b] can be either an increasing or decreasing range, min is always mapped to a, and max to b.
    unsigned char LinearInterpolationToChar(float value, float min, float max, unsigned char a, unsigned char b);

    // Construct external normal (z positive) from the X,Y components of the internal normal (z negative).
    // Synthetic normals are facing the camera (external),
    // generated normals are not (internal), this function is used to make the conversion,
    // by changing the sign of the coordinates.
    cv::Vec3f ConstructExternalNormalFromXY(float normalX, float normalY);
}
