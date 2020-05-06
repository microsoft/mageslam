// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "MageSlam.h"
#include "Image/ImageFactory.h"
#include "Device/CameraCalibration.h"

namespace UnitTests
{
    class FeatureMatcherUnitTest;
}

namespace mage
{
    class CameraCalibration;

    namespace calibration
    {
        class CameraModel;
    } 

    class ImagePreprocessor
    {
    public:
        ImagePreprocessor() = default;

        void CalculateUndistortedCalibration(const cv::Size& distortedImageSize, const CameraCalibration& distortedCameraCal, CameraCalibration& undistortedCameraCal);
        void UndistortImage(const cv::Mat& distortedImage, const CameraCalibration& distortedCameraCal, cv::Mat& undistortedImage, CameraCalibration& undistortedCameraCal);

        static bool ScaleImageForCameraConfiguration(const MAGESlam::CameraConfiguration& sourceConfig, const CameraCalibration& sourceCameraCal, const MAGESlam::CameraConfiguration& targetConfig,
            const CameraCalibration& targetCameraCal, const cv::Mat& sourceImage, const StereoSettings& stereoSettings, cv::Mat& preparedImage, CameraCalibration& preparedCameraCal, float& scaleSourceToTarget);
      
    private:

        bool CachedUndistortDataValid(const cv::Size& distortedImageSize, const CameraCalibration& distortedCameraCal) const;

        // Cache parameters for whether to detect whether to compute the undistortion map
        cv::Size m_size{};
        CameraCalibration m_distortedCameraCal;
      
        // cache parameters for the undistort to re-use
        CameraCalibration m_undistortedCameraCal;
        cv::Mat m_undistortionMap1{};
        cv::Mat m_undistortionMap2{};

        friend class ::UnitTests::FeatureMatcherUnitTest;
    };
}
