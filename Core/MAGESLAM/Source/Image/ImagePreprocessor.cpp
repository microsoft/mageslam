// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "ImagePreprocessor.h"
#include "Data/Data.h"
#include "Utils/cv.h"
#include "Utils/Logging.h"
#include "Utils/MageConversions.h"
#include "Tracking/Reprojection.h"
#include "Device/CameraCalibration.h"

#include "MageUtil.h"

#include <opencv2/imgproc.hpp>

namespace mage
{
    bool ImagePreprocessor::ScaleImageForCameraConfiguration(const MAGESlam::CameraConfiguration& sourceConfig, const CameraCalibration& sourceCameraCalibration,
                                                          const MAGESlam::CameraConfiguration& targetConfig, const CameraCalibration& targetCameraCalibration,
                                                          const cv::Mat& rawSourceImage, const StereoSettings& stereoSettings,
                                                          cv::Mat& preparedImage, CameraCalibration& preparedCameraCalibration, float& scaleSourceToTarget)
    {
        SCOPE_TIMER(ImagePreprocessor::ScaleImageForCameraConfiguration);

        assert(sourceCameraCalibration.GetDistortionType() == calibration::DistortionType::None && "expecting an undistorted source");
        assert(targetCameraCalibration.GetDistortionType() == calibration::DistortionType::None && "expecting an undistorted target");
        assert(sourceCameraCalibration.GetCalibrationWidth()  == sourceConfig.Size.Width  && "expected source calibration that matched resolution");
        assert(sourceCameraCalibration.GetCalibrationHeight() == sourceConfig.Size.Height && "expected source calibration that matched resolution");
        assert(targetCameraCalibration.GetCalibrationWidth()  == targetConfig.Size.Width  && "expected target calibration that matched resolution");
        assert(targetCameraCalibration.GetCalibrationHeight() == targetConfig.Size.Height && "expected target calibration that matched resolution");

        //calculate overlap crop (how big is the source frame in the target frame in pixels)
        //TODO: (PERF) cache this as long as targetcal and sourcecal match
        cv::Matx44f targetToSource = ToCVMat4x4(sourceConfig.Extrinsics) * ToCVMat4x4(targetConfig.Extrinsics).inv(); 
        cv::Rect targetFrameCrop{ ToCVRect(CalculateOverlapCropSourceInTarget(ToMageMat(targetToSource), targetCameraCalibration.CreateCameraModel(), sourceCameraCalibration.CreateCameraModel(), stereoSettings.StereoMapInitializationSettings.MaxDepthMeters)) };

        //if there's no overlap between the cameras, there's no way to bring them into agreement
        if (IsEntirelyOffscreen(targetFrameCrop, cv::Size{ (int)targetCameraCalibration.GetCalibrationWidth(), (int)targetCameraCalibration.GetCalibrationHeight() }))
            return false;

        //resize source image to best match the resolution of the target image
        // this is an approximation of the work stereo rectify does. if the the cameras
        // are rotated significantly to one another perspective effects will prevent there
        // from being a single scale to bring the two images into agreement. it will also
        // show a difference in aspect if the crop rect is honored explicitly. This uses the result that
        // would result in the highest resolution for the source image as a hueristic. 
        scaleSourceToTarget = std::max(targetFrameCrop.width / (float)sourceConfig.Size.Width, targetFrameCrop.height / (float)sourceConfig.Size.Height);
        cv::Size scaledSourceSize = { (int)(sourceConfig.Size.Width * scaleSourceToTarget),(int)(sourceConfig.Size.Height * scaleSourceToTarget) };
        assert(scaledSourceSize.width > 0 && scaledSourceSize.height > 0 && "not expecting a nonzero size");

        if (scaleSourceToTarget != 1.0f)
        {
            //TODO: recommended decimation is INTER_AREA, didn't change behavior from STEREO_DEMO
            //TODO: downsampled image is very noisy, may want a smooth as well if this is being done on gpu
            cv::resize(rawSourceImage, preparedImage, scaledSourceSize, 0, 0, cv::INTER_LINEAR);
            preparedCameraCalibration = CameraCalibration(std::make_shared<calibration::PinholeCameraModel>(sourceCameraCalibration.GetScaledIntrinsics(scaleSourceToTarget)));
        }
        else
        {
            preparedImage = rawSourceImage.clone(); //TODO: detect if src Mat & is same as dst mat &, skip the copy if it is in-place
            preparedCameraCalibration = sourceCameraCalibration;
        }

        return true;
    }

    bool ImagePreprocessor::CachedUndistortDataValid(const cv::Size& distortedImageSize, const CameraCalibration& distortedCameraCal) const
    {
        return (distortedImageSize == m_size) && (distortedCameraCal == m_distortedCameraCal);
    }

    void ImagePreprocessor::CalculateUndistortedCalibration(const cv::Size& distortedImageSize, const CameraCalibration& distortedCameraCal, CameraCalibration& undistortedCameraCal)
    {
        SCOPE_TIMER(ImagePreprocessor::CalculateUndistortedCalibration);
        assert(distortedCameraCal.GetDistortionType() != calibration::DistortionType::None && "Don't need to call this with undistorted camera model");

        if (CachedUndistortDataValid(distortedImageSize, distortedCameraCal))
        {
            undistortedCameraCal = m_undistortedCameraCal;
        }
        else
        {
            //save values that describe what the cached maps are for
            m_distortedCameraCal = distortedCameraCal;
            m_size = distortedImageSize;

            // rather than calculate optimal matrix based on the undistortion (zoom out or zoom in and crop), use the distorted FX/FY as is, and call the center of the image the principle point
            // a test against BVT shows this approach gets better results, work item to find out why: Deliverable 14998587
            cv::Matx33f undistortedCameraMatrix = m_distortedCameraCal.GetCameraMatrix();
            undistortedCameraMatrix(0, 2) = m_size.width * 0.5f;
            undistortedCameraMatrix(1, 2) = m_size.height * 0.5f;

            Intrinsics undistortedIntrinsics{
                undistortedCameraMatrix(0,2), //cx
                undistortedCameraMatrix(1,2), //cy
                undistortedCameraMatrix(0,0), //fx
                undistortedCameraMatrix(1,1), //fy
                (uint32_t)m_size.width,
                (uint32_t)m_size.height };

            m_undistortedCameraCal = CameraCalibration(std::make_shared<calibration::PinholeCameraModel>(undistortedIntrinsics));
            undistortedCameraCal = m_undistortedCameraCal;
        }
    }
    
    void ImagePreprocessor::UndistortImage(const cv::Mat& distortedImage, const CameraCalibration& distortedCameraCal, cv::Mat& undistortedImage, CameraCalibration& undistortedCameraCal)
    {
        SCOPE_TIMER(ImagePreprocessor::UndistortImage);
        assert(distortedCameraCal.GetDistortionType() != calibration::DistortionType::None && "Don't need to call this with undistorted camera model");
        
        if (!CachedUndistortDataValid(distortedImage.size(), m_distortedCameraCal))
        {
            CalculateUndistortedCalibration(distortedImage.size(), distortedCameraCal, undistortedCameraCal);
            cv::initUndistortRectifyMap(distortedCameraCal.GetCameraMatrix(), distortedCameraCal.GetCVDistortionCoeffs(),
                cv::noArray(), m_undistortedCameraCal.GetCameraMatrix(), m_size, CV_32FC1, m_undistortionMap1, m_undistortionMap2);
        }

        undistortedCameraCal = m_undistortedCameraCal;
        cv::remap(distortedImage, undistortedImage, m_undistortionMap1, m_undistortionMap2, cv::INTER_LINEAR);
    }
    
}
