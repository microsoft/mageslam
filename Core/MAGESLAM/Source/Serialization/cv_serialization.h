// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana\utils\serialization\custom_serialization.h"

#include <opencv2\core.hpp>

namespace mira
{
    template<>
    struct custom_serialization<cv::Point3f>
    {
        template<typename StreamT>
        static void read(StreamT& stream, cv::Point3f& element)
        {
            read_multiple<3>(stream, &element.x);
        }

        template<typename StreamT>
        static void write(StreamT& stream, const cv::Point3f& element)
        {
            write_multiple<3>(stream, &element.x);
        }
    };

    template<size_t Rows, size_t Cols>
    struct custom_serialization<cv::Matx<float, Rows, Cols>>
    {
        template<typename StreamT>
        static void read(StreamT& stream, cv::Matx<float, Rows, Cols>& element)
        {
            read_multiple<Rows * Cols>(stream, element.val);
        }

        template<typename StreamT>
        static void write(StreamT& stream, const cv::Matx<float, Rows, Cols>& element)
        {
            write_multiple<Rows * Cols>(stream, element.val);
        }
    };

    template<>
    struct custom_serialization<cv::Vec3f> : public custom_serialization<cv::Vec3f::mat_type>
    {};

    template<>
    struct custom_serialization<cv::Vec4f> : public custom_serialization<cv::Vec4f::mat_type>
    {};
}