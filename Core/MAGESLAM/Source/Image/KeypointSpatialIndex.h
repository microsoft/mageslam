// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// KeypointSpatialIndex.h
//
// Allows to construction, query, and removal of Keypoint entries based on pixel position.
//------------------------------------------------------------------------------

#pragma once

#include "Utils\thread_memory.h"

#include <memory>
#include <vector>
#include <functional>
#include <opencv2\features2d\features2d.hpp>

#include <gsl/span>

namespace mage
{
    class KeypointSpatialIndex
    {
        struct Impl;

    public:
        KeypointSpatialIndex(gsl::span<const cv::KeyPoint> keypoints);

        void Query(const cv::Point2f& center, int octave, float radius, temp::vector<size_t>& results) const;
        void Remove(const cv::KeyPoint& keypoint, size_t value);

        ~KeypointSpatialIndex();

    private:
        static constexpr float octaveSpacing = 100;
        static constexpr float octaveQueryRange = 1;

        std::unique_ptr<Impl> m_impl;
    };
}