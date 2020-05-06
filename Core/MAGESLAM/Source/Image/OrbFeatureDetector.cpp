// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// OrbFeatureDetector.cpp
//
// The feature detector detects the orb features in an image and 
// also computes the  BRIEF descriptors for them. The init settings 
// control the properties of the features.
//------------------------------------------------------------------------------


#include "OrbFeatureDetector.h"
#include "MageSettings.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "Utils/Logging.h"
#include "OpenCVModified.h"

#include "Analysis/DataFlow.h"

#include <gsl/gsl_algorithm>

namespace mage
{    
    void OrbFeatureDetector::UndistortKeypoints(
        gsl::span<cv::KeyPoint> inoutKeypoints,
        const CameraCalibration& distortedCalibration,
        const CameraCalibration& undistortedCalibration,
        thread_memory memory)
    {       
        SCOPE_TIMER(OrbFeatureDetector::UndistortKeypoints);

        assert(distortedCalibration.GetDistortionType() != mage::calibration::DistortionType::None && "expecting distorted keypoints to undistort");

        if (inoutKeypoints.empty())
        {
            return;
        }

        auto sourceMem = memory.stack_buffer<cv::Point2f>(inoutKeypoints.size());
        for (size_t i = 0; i < sourceMem.size(); ++i)
        {
            sourceMem[i] = inoutKeypoints[i].pt;
        }
        cv::Mat distortedPointsMat{ (int)sourceMem.size(), 1, CV_32FC2, sourceMem.data() };

        auto destMem = memory.stack_buffer<cv::Point2f>(inoutKeypoints.size());
        cv::Mat undistortedPointsMat{ (int)destMem.size(), 1, CV_32FC2, destMem.data() };
     
        undistortPoints(distortedPointsMat, undistortedPointsMat, distortedCalibration.GetCameraMatrix(),  distortedCalibration.GetCVDistortionCoeffs(), cv::noArray(), undistortedCalibration.GetCameraMatrix());

        // now adjust the points in undistorted
        for (size_t i = 0; i < destMem.size(); ++i)
        {
            inoutKeypoints[i].pt = destMem[i];
        }
    }

    OrbFeatureDetector::OrbFeatureDetector(const FeatureExtractorSettings& settings)
        : m_detector{
        settings.GaussianKernelSize,
        (unsigned int)settings.NumFeatures,
        settings.ScaleFactor,
        settings.NumLevels,
        settings.PatchSize,
        settings.FastThreshold,
        settings.UseOrientation,
        settings.FeatureFactor,
        settings.FeatureStrength,
        settings.StrongResponse,
        settings.MinRobustnessFactor,
        settings.MaxRobustnessFactor,
        settings.NumCellsX,
        settings.NumCellsY
    }
    {
    }

    void OrbFeatureDetector::Process(const CameraCalibration& distortedCalibration, const CameraCalibration& undistortedCalibration, thread_memory memory, ImageHandle& imageData, const cv::Mat& image)
    {
        SCOPE_TIMER(OrbFeatureDetector::Process);

        assert(undistortedCalibration.GetDistortionType() == calibration::DistortionType::None && "Expecting undistorted cal to be undistorted");

        DATAFLOW(
            DF_OUTPUT(imageData->GetKeypoints())
        );

        m_detector.DetectAndCompute(memory, *imageData, image);

        if(distortedCalibration != undistortedCalibration)
        {
            UndistortKeypoints(imageData->GetKeypoints(), distortedCalibration, undistortedCalibration, memory);
        }
    }
}
