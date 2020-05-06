/*
We are using the OpenCV implementation and adding gridding to the 
feature detection. 

Modified : computeKeyPoints
Added methods : keepBestKeyPoints and subimageToImageCoordinates
*/


/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** Authors: Ethan Rublee, Vincent Rabaud, Gary Bradski */


#pragma once

#include "Utils\thread_memory.h"
#include "ImageData.h"

#include <iterator>

#include <opencv2\core\types.hpp>

#include <gsl\gsl>

namespace UnitTests
{
    class OpenCVModifiedUnitTest;
    class OrbFeatureDetectorUnitTest;
}

class OrbDetector
{
    using ImageData = mage::ImageData<mage::ImageAllocator>;
public:
    OrbDetector(
        unsigned int gaussianKernelSize,
        unsigned int nfeatures,
        float scaleFactor,
        unsigned int nlevels,
        unsigned int patchSize,
        unsigned int fastThreshold,
        bool useOrientation,
        float featureFactorANMS,
        float featureStrengthANMS,
        int strongResponseANMS,
        float minRobustFactor,
        float maxRobustFactor,
        int numCellsX,
        int numCellsY);

    // Compute the ORB_Impl features and descriptors on an image
    void DetectAndCompute(
        mage::thread_memory memory,
        ImageData& imageData,
        const cv::Mat& image);
private:

    void ComputeKeyPoints(const cv::Mat& imagePyramid,
        gsl::span<const cv::Rect> layerInfo,
        gsl::span<const float> layerScale,
        mage::thread_memory memory,
        ImageData& result);
    
    static void ResizeAndComputeOrbDescriptorsPrerotated(
        const cv::Mat& image,
        gsl::span<const cv::KeyPoint> keypoints,
        gsl::span<mage::ORBDescriptor> descriptors,
        const std::vector<std::vector<signed char>>& scaledPatterns,
        size_t dsize,
        float upScale);    // amount to upscale the source image


    static void ComputeOrbDescriptorsPrerotated(
        const cv::Mat& imagePyramid,
        gsl::span<const cv::Rect> layerInfo,
        gsl::span<const float> layerScale,
        gsl::span<const cv::KeyPoint> keypoints,
        gsl::span<mage::ORBDescriptor> descriptors,
        const signed char* _patternRotated,
        size_t dsize);

    static void ComputeOrbDescriptors(
        const cv::Mat& imagePyramid,
        gsl::span<const cv::Rect> layerInfo,
        gsl::span<const float> layerScale,
        gsl::span<const cv::KeyPoint> keypoints,
        gsl::span<mage::ORBDescriptor> descriptors,
        gsl::span<const cv::Point> _pattern,
        size_t dsize);

    static void ICAngles(
        const cv::Mat& img,
        gsl::span<const cv::Rect> layerinfo,
        gsl::span<const int> u_max,
        int half_k,
        gsl::span<cv::KeyPoint> pts);

    static void HarrisResponses(
        const cv::Mat& img,
        gsl::span<const cv::Rect> layerinfo,
        gsl::span<cv::KeyPoint> pts,
        int blockSize,
        float harris_k);

    static void MakeRandomPattern(int patchSize, cv::Point* pattern, int npoints);

    enum class FeatureType
    {
        TYPE_5_8 = 0, TYPE_7_12 = 1, TYPE_9_16 = 2
    };

    void AdaptiveNonMaximalSuppresion(mage::temp::vector<cv::KeyPoint>& keypoints,
        unsigned int numToKeep, int threshold, mage::thread_memory memory);
    void FAST(
        const cv::Mat& img,
        mage::temp::vector<cv::KeyPoint>& keypoints,
        int threshold,
        bool nonmax_suppression,
        uchar* threshold_tab,
        mage::thread_memory& memory,
        FeatureType type = FeatureType::TYPE_9_16);

    const unsigned int m_gaussianKernelSize;
    const unsigned int m_nfeatures;
    const float m_scaleFactor;
    const unsigned int m_nlevels;
    const unsigned int m_patchSize;
    const unsigned int m_fastThreshold;
    const bool m_useOrientation;
    const float m_featureFactorANMS;
    const float m_featureStrengthANMS;
    const int m_strongResponseANMS;
    const float m_minRobustFactorANMS;
    const float m_maxRobustFactorANMS;
    const int m_numCellsX;
    const int m_numCellsY;

    friend class ::UnitTests::OpenCVModifiedUnitTest;
    friend class ::UnitTests::OrbFeatureDetectorUnitTest;
};

