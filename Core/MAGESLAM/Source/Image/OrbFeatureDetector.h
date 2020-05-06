// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// OrbFeatureDetector.h
//
// The feature detector detects the orb features in an image and 
// also computes the  BRIEF descriptors for them. The init settings 
// control the properties of the features.
//------------------------------------------------------------------------------

#pragma once

#include "MageSettings.h"
#include "ORBDescriptor.h"
#include "Device/CameraCalibration.h"
#include "OpenCVModified.h"

#include "Utils\thread_memory.h"

#include "ImageFactory.h"

#include <opencv2\core\types.hpp>

namespace UnitTests
{
    class OrbFeatureDetectorUnitTest;
}

namespace mage
{
    class OrbFeatureDetector
    {
    public:
        OrbFeatureDetector(const FeatureExtractorSettings& settings);
      
        void Process(const CameraCalibration& distortedCalibration, const CameraCalibration& undistortedCalibration, thread_memory memory, ImageHandle& imageData, const cv::Mat& image);

    private:
        
        OrbDetector m_detector;
        
        void UndistortKeypoints(
            gsl::span<cv::KeyPoint> inoutKeypoints,
            const CameraCalibration& distortedCalibration,
            const CameraCalibration& undistortedCalibration,
            thread_memory memory);

        friend class ::UnitTests::OrbFeatureDetectorUnitTest;
    };
}
