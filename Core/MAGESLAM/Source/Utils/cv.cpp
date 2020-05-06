// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "cv.h"

namespace mage
{
    cv::Mat CreateGrayCVMat(const cv::Size& resolution, const PixelFormat& format, const gsl::span<const uint8_t> imageBytes)
    {
        switch (format)
        {
        case PixelFormat::GRAYSCALE8:
        {
            const auto data = const_cast<void*>(static_cast<const void*>(imageBytes.data()));
            return cv::Mat(resolution, CV_8UC1, data).clone(); // TODO: to share memory between caller and mage
        }
        break;
        case PixelFormat::NV12:
        {
            const auto data = const_cast<void*>(static_cast<const void*>(imageBytes.subspan(0, resolution.width * resolution.height).data()));
            return cv::Mat(resolution, CV_8UC1, data).clone(); // TODO: to share memory between caller and mage
        }
        break;
        default:
            assert(false && "unsupported pixel format");
            return {};
        }
    }

    cv::Mat CreateBGRCVMat(const cv::Mat& source, const PixelFormat& format)
    {
        cv::Mat dst;

        switch (format)
        {
        case PixelFormat::GRAYSCALE8:
        {
            cv::cvtColor(source, dst, CV_GRAY2BGR);
        }
        break;
        case PixelFormat::NV12:
        {
            cv::cvtColor(source, dst, CV_YUV2BGR_NV12);
        }
        break;
        default:
            assert(false && "unsupported pixel format");
            break;
        }

        return dst;
    }
}
