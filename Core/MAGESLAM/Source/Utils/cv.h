// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data/Data.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <assert.h>
#include <array>
#include <gsl/gsl>

#include "eigen.h"

namespace mage
{
    using Quaternion = Eigen::Quaternionf;

    inline Quaternion ToQuat(const cv::Matx33f& rotation)
    {
        Eigen::Matrix3f eigenMat;
        cv::cv2eigen(rotation, eigenMat);

        Quaternion eigenQuat(eigenMat);
        // TODO validate if this normalize is even necessary, eigen might already be doing this
        eigenQuat.normalize();

        return eigenQuat;
    }

    template<int N>
    inline Eigen::Map<Eigen::Matrix<float, N, 1>> ToMap(cv::Vec<float, N>& vec)
    {
        return Eigen::Map<Eigen::Vector3f>{ vec.val };
    }

    template<int N>
    inline Eigen::Map<const Eigen::Matrix<float, N, 1>> ToCMap(const cv::Vec<float, N>& vec)
    {
        return Eigen::Map<const Eigen::Matrix<float, N, 1>>{ vec.val };
    }

    inline Eigen::Map<Eigen::Vector3f> ToMap(cv::Matx31f& vec)
    {
        return Eigen::Map<Eigen::Vector3f>{ vec.val };
    }

    inline Eigen::Map<const Eigen::Vector3f> ToCMap(const cv::Matx31f& vec)
    {
        return Eigen::Map<const Eigen::Vector3f>{ vec.val };
    }

    inline Eigen::Map<Eigen::Vector3f> ToMap(cv::Point3f& vec)
    {
        return Eigen::Map<Eigen::Vector3f>{ &vec.x };
    }

    inline Eigen::Map<const Eigen::Vector3f> ToCMap(const cv::Point3f& vec)
    {
        return Eigen::Map<const Eigen::Vector3f>{ &vec.x };
    }

    inline Eigen::Map<Eigen::Vector2f> ToMap(cv::Point2f& vec)
    {
        return Eigen::Map<Eigen::Vector2f>{ &vec.x };
    }

    inline Eigen::Map<const Eigen::Vector2f> ToCMap(const cv::Point2f& vec)
    {
        return Eigen::Map<const Eigen::Vector2f>{ &vec.x };
    }

    template<size_t N>
    inline cv::Vec<float, N> ToVec(gsl::span<const float, N> values)
    {
        return cv::Vec<float, N>(values.data());
    }

    inline Quaternion QuatFromTwoVectors(const cv::Vec3f& from, const cv::Vec3f& to)
    {
        Quaternion quat;
        quat.setFromTwoVectors(Eigen::Vector3f{ from(0), from(1), from(2) }, Eigen::Vector3f{ to(0), to(1), to(2) });
        return quat;
    }

    inline cv::Matx33f ToMat(const Quaternion& quat)
    {
        cv::Matx33f mat;
        cv::eigen2cv(quat.toRotationMatrix(), mat);
        return mat;
    }

    inline cv::Matx33f ToMat(float pitchRads, float rollRads, float yawRads)
    {
        // Formula for (Pitch * Roll * Yaw) from http://www.songho.ca/opengl/gl_anglestoaxes.html
        float a = pitchRads;
        float b = yawRads;
        float c = rollRads;

        float sa = sinf(a);
        float sb = sinf(b);
        float sc = sinf(c);

        float ca = cosf(a);
        float cb = cosf(b);
        float cc = cosf(c);

        return cv::Matx33f{
            cc * cb,                  -sc,         cc * sb,
            ca * sc * cb + sa * sb,   ca * cc,     ca * sc * sb - sa * cb,
            sa * sc * cb - ca * sb,   sa * cc,     sa * sc * sb + ca * cb
        };
    }

    inline cv::Matx44f To4x4(const cv::Matx34f& mat)
    {
        return{
            mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
            mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
            mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3),
            0,         0,         0,         1
        };
    }

    inline cv::Matx34f To3x4(const cv::Matx44f& mat)
    {
        return{
            mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
            mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
            mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3)
        };
    }

    inline cv::Matx44f To4x4(const cv::Matx33f& rot, const cv::Vec3f& trans)
    {
        return{
            rot(0, 0), rot(0, 1), rot(0, 2), trans(0),
            rot(1, 0), rot(1, 1), rot(1, 2), trans(1),
            rot(2, 0), rot(2, 1), rot(2, 2), trans(2),
            0,         0,         0,         1
        };
    }

    inline cv::Matx44f FromQuatAndTrans(const Quaternion& quat, const cv::Vec3f& trans)
    {
        return To4x4(ToMat(quat), trans);
    }

    inline cv::Matx33f Rotation(const cv::Matx44f& mat)
    {
        return{
            mat(0, 0), mat(0, 1), mat(0, 2),
            mat(1, 0), mat(1, 1), mat(1, 2),
            mat(2, 0), mat(2, 1), mat(2, 2)
        };
    }

    inline cv::Matx33f Rotation(const cv::Matx34f& mat)
    {
        return{
            mat(0, 0), mat(0, 1), mat(0, 2),
            mat(1, 0), mat(1, 1), mat(1, 2),
            mat(2, 0), mat(2, 1), mat(2, 2)
        };
    }

    inline cv::Vec3f Translation(const cv::Matx44f& mat)
    {
        return{
            mat(0, 3),
            mat(1, 3),
            mat(2, 3)
        };
    }

    inline cv::Vec3f Translation(const cv::Matx34f& mat)
    {
        return{
            mat(0, 3),
            mat(1, 3),
            mat(2, 3)
        };
    }
	//Swap the returned parameters to suppress x64 compile warning c4324
	//c:\program files(x86)\microsoft visual studio\2017\enterprise\vc\tools\msvc\14.11.25503\include\utility(271) : warning C4324 : 'std::pair<mage::Quaternion,cv::Vec3f>' : structure was padded due to alignment specifier
    inline std::pair<cv::Vec3f, Quaternion> Decompose(const cv::Matx44f& matrix)
    {
        return std::make_pair(Translation(matrix), ToQuat(matrix.get_minor<3, 3>(0, 0)));
    }

    template<int ROWS, int COLS>
    inline bool MatxEqual(const cv::Matx<float, ROWS, COLS>& mat0, const cv::Matx<float, ROWS, COLS>& mat1)
    {
        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                if (mat0(i,j) != mat1(i,j))
                    return false;
            }
        }

        return true;
    }

    inline bool MatEqual(const cv::Mat& mat0, const cv::Mat& mat1)
    {
        if (mat0.size != mat1.size)
            return false;

        for (int i = 0; i < mat0.rows; i++)
        {
            for (int j = 0; j < mat0.cols; j++)
            {
                if (mat0.at<float>(i, j) != mat1.at<float>(i, j))
                    return false;
            }
        }

        return true;
    }

    template<int ROWS, int COLS>
    inline cv::Matx44f Invert(const cv::Matx<float, ROWS, COLS>& transform)
    {
        static_assert((ROWS == 3 || ROWS == 4) && COLS == 4, "Invert only supports 3x4 or 4x4 matrices");

        //invert rotation by transpose on copy of upper 3x3 for inverse rotation       
        cv::Matx44f invRotation{
            transform(0, 0), transform(1, 0), transform(2, 0), 0,
            transform(0, 1), transform(1, 1), transform(2, 1), 0,
            transform(0, 2), transform(1, 2), transform(2, 2), 0,
                          0,               0,               0, 1
        };

#ifndef NDEBUG
        // this approach would only work if the upper 3x3 was a pure rotation matrix (no scale)
        // that would mean all rows and columns were unit length and orthogonal. this tests for the former.
        for(int idx=0; idx < 3; idx++)
        { 
            assert(abs(1.0 - cv::norm(invRotation.col(idx))) <= 0.001 && "Expecting unit length (no scale) for rotation matrix");
            assert(abs(1.0 - cv::norm(invRotation.row(idx))) <= 0.001 && "Expecting unit length (no scale) for rotation matrix");
        }

        //a valid rotation matrix has determinant 1 (a mirror would be -1)
        assert(abs(1.0 - cv::determinant(invRotation)) < 0.001  && "Expecting determinant of 1 for rotation matrix");
#endif


        //invert translation by negation on copy
        cv::Matx44f invTranslation{
            1, 0, 0, -transform(0, 3),
            0, 1, 0, -transform(1, 3),
            0, 0, 1, -transform(2, 3),
            0, 0, 0,               1
        };

        //inverse matrix
        return invRotation * invTranslation;
    }

    inline cv::Matx44f ComputeFrameTransform(const cv::Matx44f& from, const cv::Matx44f& to)
    {
        return to * Invert(from);
    }

    inline cv::Point3f UnProject(const cv::Matx33f& invCameraMatrix, const cv::Matx44f& viewMatrix, const cv::Point2i& point, float depth)
    {
        cv::Matx31f pixelSpace{ static_cast<float>(point.x), static_cast<float>(point.y), 1.f };
        cv::Matx31f cameraSpace = invCameraMatrix * pixelSpace;
        cameraSpace *= depth;

        cv::Matx41f world = Invert(viewMatrix) * cv::Matx41f{ cameraSpace(0), cameraSpace(1), cameraSpace(2), 1 };
        return{ world(0), world(1), world(2) };
    }

    const float NRM_EPSILON = 0.00001f;

    inline float Length(const cv::Vec3f& vec)
    {
        return std::sqrtf(vec.dot(vec));
    }

    inline cv::Vec3f Normalize(const cv::Vec3f& vec)
    {        
        float distance = Length(vec);
        if (distance == 0)
            return vec;

        cv::Vec3f normVec = (vec / distance);
        assert(abs(sqrt(normVec.dot(normVec)) - 1.0f) < NRM_EPSILON);
        return normVec;
    }

    //rightHanded, column vec convention, intended for rotation of vectors (active)
    //https://en.wikipedia.org/wiki/Rotation_matrix
    //vs http://mathworld.wolfram.com/RotationMatrix.html intended for rotation of coordinate frames
    inline cv::Matx44f RotationXForVectors(float radians)
    {
        float cosR = std::cosf(radians);
        float sinR = std::sinf(radians);

        return{ 1.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, cosR, -sinR, 0.0f,
                 0.0f, sinR, cosR, 0.0f,
                 0.0f, 0.0f, 0.0f, 1.0f };
    }

    //rightHanded, column vec convention, intended for rotation of vectors (active)
    //https://en.wikipedia.org/wiki/Rotation_matrix
    //vs http://mathworld.wolfram.com/RotationMatrix.html intended for rotation of coordinate frames
    inline cv::Matx44f RotationYForVectors(float radians)
    {
        float cosR = std::cosf(radians);
        float sinR = std::sinf(radians);

        return{ cosR, 0.0f, sinR, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                -sinR, 0.0f, cosR, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f };
    }

    //rightHanded, column vec convention, intended for rotation of vectors (active)
    //https://en.wikipedia.org/wiki/Rotation_matrix
    //vs http://mathworld.wolfram.com/RotationMatrix.html intended for rotation of coordinate frames
    inline cv::Matx44f RotationZForVectors(float radians)
    {
        float cosR = std::cosf(radians);
        float sinR = std::sinf(radians);

        return{ cosR, -sinR, 0.0f, 0.0f,
                sinR, cosR, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f };
                
    }

    //rightHanded, column vec convention, intended for rotation of coordinate frames (passive)
    //https://en.wikipedia.org/wiki/Rotation_matrix
    //ala http://mathworld.wolfram.com/RotationMatrix.html 
    inline cv::Matx44f RotationXForCoordinateFrames(float radians)
    {
        float cosR = std::cosf(radians);
        float sinR = std::sinf(radians);

        return{ 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, cosR, sinR, 0.0f,
            0.0f, -sinR, cosR, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f };
    }

    //rightHanded, column vec convention,intended for rotation of coordinate frames (passive)
    //https://en.wikipedia.org/wiki/Rotation_matrix
    //ala http://mathworld.wolfram.com/RotationMatrix.html 
    inline cv::Matx44f RotationYForCoordinateFrames(float radians)
    {
        float cosR = std::cosf(radians);
        float sinR = std::sinf(radians);

        return{ cosR, 0.0f, -sinR, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            sinR, 0.0f, cosR, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f };
    }

    //rightHanded, column vec convention, intended for rotation of coordinate frames (passive)
    //https://en.wikipedia.org/wiki/Rotation_matrix
    //ala http://mathworld.wolfram.com/RotationMatrix.html 
    inline cv::Matx44f RotationZForCoordinateFrames(float radians)
    {
        float cosR = std::cosf(radians);
        float sinR = std::sinf(radians);

        return{ cosR, sinR, 0.0f, 0.0f,
            -sinR, cosR, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f };
    }

    // encoding of axis/angle in a vec3f
    // http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues
    inline cv::Matx33f RotationFromRodrigues(const cv::Matx31f& rotation)
    {
        cv::Matx33f rotationMat;
        cv::Rodrigues(rotation, rotationMat);
        return rotationMat;
    }

    template<typename T, size_t M, size_t N>
    inline std::array<T, M*N> ArrayFromMat(const cv::Matx<T, M, N>& mat)
    {
        std::array<T, M * N> out;
        std::copy(std::begin(mat.val), std::end(mat.val), out.begin());
        return out;
    }

    inline cv::Matx44f ToCVMat4x4(const mage::Matrix& matrix)
    {
        return cv::Matx44f(&matrix.M11);
    }

    inline bool IsEntirelyOffscreen(const cv::Rect& rect, const cv::Size& screenResPixels)
    {
        assert(rect.width > 0 && "invalid rect");
        assert(rect.height > 0 && "invalid rect");
        assert(screenResPixels.width > 0 && "invalid screenResPixels");
        assert(screenResPixels.height > 0 && "invalid screenResPixels");

        int maxHorizPixel = rect.width + rect.x - 1;
        int maxVerticalPixel = rect.height + rect.y - 1;

        bool offscreenX = (maxHorizPixel < 0) || (rect.x >(int)(screenResPixels.width - 1));
        bool offscreenY = (maxVerticalPixel < 0) || (rect.y >(int)(screenResPixels.height - 1));

        return (offscreenX || offscreenY);
    }

    inline bool IsWithinArea(const cv::Point2f& pt, const cv::Rect& crop)
    {
        assert(crop.width > 0 && "width must be positive nonzero");
        assert(crop.height > 0 && "height must be positive nonzero");

        return (pt.x >= crop.x && pt.x < (crop.x+crop.width)) && (pt.y >= crop.y && pt.y <= (crop.y + crop.height));
    }

    inline cv::Rect ToCVRect(const mage::Rect& rect)
    {
        return { (int)rect.X, (int)rect.Y, (int)rect.Width, (int)rect.Height };
    }

    cv::Mat CreateGrayCVMat(const cv::Size& resolution, const PixelFormat& format, const gsl::span<const uint8_t> imageBytes);
    cv::Mat CreateBGRCVMat(const cv::Mat& source, const PixelFormat& format);
}
