// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MageConversions.h"
#include <arcana/math.h>
#include <array>

namespace mage
{
    bool IsIdentity(const Matrix& m)
    {
        return (m.M11 == 1.0f && m.M12 == 0.0f && m.M13 == 0.0f && m.M14 == 0.0f) &&
            (m.M21 == 0.0f && m.M22 == 1.0f && m.M23 == 0.0f && m.M24 == 0.0f) &&
            (m.M31 == 0.0f && m.M32 == 0.0f && m.M33 == 1.0f && m.M34 == 0.0f) &&
            (m.M41 == 0.0f && m.M42 == 0.0f && m.M43 == 0.0f && m.M44 == 1.0f);
    }

    Matrix ToMageMat(const cv::Matx34f& cvMat)
    {
        Matrix outputMat{};
        outputMat.M44 = 1.f;
        memcpy(&outputMat, cvMat.val, sizeof(cvMat.val));
        return outputMat;
    }

    Matrix ToMageMat(const cv::Matx44f& cvMat)
    {
        static_assert(sizeof(Matrix) == sizeof(cvMat.val), "sizes should be equal, or else initialize to zeros");

        Matrix outputMat;
        memcpy(&outputMat, cvMat.val, sizeof(Matrix));
        return outputMat;
    }

    Position ToMagePos(const cv::Point3f& cvPt)
    {
        return{ cvPt.x, cvPt.y, cvPt.z };
    }

    Direction ToMageDir(const cv::Point3f& cvDir)
    {
        return{ cvDir.x, cvDir.y, cvDir.z };
    }

    cv::Vec3f FromMageDir(const Direction& mageDir)
    {
        return { mageDir.X, mageDir.Y, mageDir.Z };
    }

    mage::Pose MageMatrixToMagePose(const mage::Matrix& viewMatrix)
    {
        cv::Matx31f viewPosition{
            viewMatrix.M14,
            viewMatrix.M24,
            viewMatrix.M34
        };

        cv::Matx33f viewRotation{
            viewMatrix.M11, viewMatrix.M12, viewMatrix.M13,
            viewMatrix.M21, viewMatrix.M22, viewMatrix.M23,
            viewMatrix.M31, viewMatrix.M32, viewMatrix.M33
        };

        return mage::Pose(viewPosition, viewRotation);
    }

    Matrix ToMageMat(const std::array<float, 4 * 4>& f)
    {
        return mage::Matrix{
            f[0],  f[1],  f[2] , f[3],
            f[4],  f[5],  f[6] , f[7],
            f[8],  f[9],  f[10], f[11],
            f[12], f[13], f[14], f[15],
        };
    }

    Direction ToMageDir(const std::array<float, 3>& dir)
    {
        return mage::Direction{ dir[0],dir[1],dir[2] };
    }

    void ConvertBGRToNV12(const cv::Mat& srcImg, gsl::span<uint8_t> destBuffer)
    {
        auto imgSize = srcImg.size();
        auto rowStride = imgSize.width;
        uint8_t* pDstBuffer = destBuffer.data();
        const uint8_t* pSrcBuffer = srcImg.data;
        uint32_t inputStride = imgSize.width * srcImg.channels();

        assert(destBuffer.size_bytes() >= imgSize.width * imgSize.height * 3 / 2);

        // Y
        for (int y = 0; y < imgSize.height; ++y, pDstBuffer += rowStride, pSrcBuffer += inputStride)
        {
            const uint8_t* pSrcPixel = pSrcBuffer;
            uint8_t* pDstPixel = pDstBuffer;
            for (int x = 0; x < imgSize.width; ++x, pSrcPixel += srcImg.channels(), pDstPixel += 1)
            {
                float Y = (0.257f*pSrcPixel[2]) + (0.504f*pSrcPixel[1]) + (0.098f*pSrcPixel[0]) + 16.0f;
                *pDstPixel = static_cast<uint8_t>(mira::clamp<float>(Y, 0.0f, 255.0f));
            }
        }

        pSrcBuffer = srcImg.data;
        //Cb, Cr
        for (int y = 0; y < imgSize.height / 2; ++y, pDstBuffer += rowStride, pSrcBuffer += 2 * inputStride)
        {
            const uint8_t* pSrcPixel = pSrcBuffer;
            uint8_t* pDstPixel = pDstBuffer;
            for (int x = 0; x < imgSize.width / 2; x++, pSrcPixel += 2 * srcImg.channels(), pDstPixel += 2)
            {
                const uint8_t* pSrcPixel0 = pSrcPixel;
                const uint8_t* pSrcPixel1 = pSrcPixel0 + srcImg.channels();
                const uint8_t* pSrcPixel2 = pSrcPixel0 + inputStride;
                const uint8_t* pSrcPixel3 = pSrcPixel1 + inputStride;

                float R = (pSrcPixel0[2] + pSrcPixel1[2] + pSrcPixel2[2] + pSrcPixel3[2]) / 4.0f;
                float G = (pSrcPixel0[1] + pSrcPixel1[1] + pSrcPixel2[1] + pSrcPixel3[1]) / 4.0f;
                float B = (pSrcPixel0[0] + pSrcPixel1[0] + pSrcPixel2[0] + pSrcPixel3[0]) / 4.0f;

                float Cr = (0.439f * R) - (0.368f * G) - (0.071f * B) + 128.0f;
                float Cb = -(0.148f * R) - (0.291f * G) + (0.439f * B) + 128.0f;

                pDstPixel[0] = static_cast<uint8_t>(mira::clamp<float>(Cb, 0.0f, 255.0f));
                pDstPixel[1] = static_cast<uint8_t>(mira::clamp<float>(Cr, 0.0f, 255.0f));
            }
        }
    }

    unsigned char LinearInterpolationToChar(float value, float min, float max, unsigned char a, unsigned char b)
    {
        if (value <= min)
        {
            return a;
        }

        if (value >= max)
        {
            return b;
        }

        float interpolatedValue = (b - a)*(value - min) / (max - min) + a;

        // Check the value is within the given range.
        assert((interpolatedValue >= a && interpolatedValue <= b) ||
            (interpolatedValue <= a && interpolatedValue >= b) &&
            "Interpolated value is not within the range");

        return static_cast<unsigned char> (interpolatedValue);
    }

    cv::Vec3f ConstructExternalNormalFromXY(float normalX, float normalY)
    {
        cv::Vec3f externalNormal;
        float dotXY = std::min(1.0f, normalX*normalX + normalY * normalY);

        externalNormal[0] = -normalX;
        externalNormal[1] = -normalY;
        externalNormal[2] = sqrt(1 - dotXY);

        return externalNormal;
    }
}
