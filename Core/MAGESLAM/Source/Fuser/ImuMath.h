// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data/Pose.h"

#include <Eigen/Core>

#include <MathLib/SO3.h>
#include <MathLib/SE3.h>

namespace mage
{
    namespace imu
    {
        template<typename T>
        using Matrix = Eigen::Matrix<T, 4, 4, Eigen::RowMajor>;

        template<typename T>
        using ViewMatrix = Eigen::Matrix<T, 3, 4, Eigen::RowMajor>;

        template<typename T>
        using RotationMatrix = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;

        template<typename T>
        using Vector = Eigen::Matrix<T, 3, 1>;

        inline ST::SE3<double> ToSE3(Eigen::Map<const Matrix<float>> transform)
        {
            ST::SE3<double> se3{};

            Eigen::Map<RotationMatrix<double>>(se3.R) = transform.block<3, 3>(0, 0).cast<double>();
            Eigen::Map<Vector<double>>(se3.t) = transform.block<3, 1>(0, 3).cast<double>();

            ST::SE3_coerce_rotation(se3);

            return se3;
        }

        inline ST::SE3<double> ToSE3(const Matrix<float>& transform)
        {
            return ToSE3(Eigen::Map<const Matrix<float>>(transform.data(), 4, 4));
        }

        inline Matrix<float> ToMatrix(const ST::SE3<double>& se3)
        {
            Eigen::Map<const RotationMatrix<double>> r{ se3.R };
            Eigen::Map<const Vector<double>> t{ se3.t };

            Matrix<float> pose = Matrix<float>::Identity();

            pose.block<3, 3>(0, 0) = r.cast<float>();
            pose.block<3, 1>(0, 3) = t.cast<float>();

            return pose;
        }

        inline ST::SE3<double> PoseToViewSE3(const Pose& pose)
        {
            imu::Matrix<float> viewMatrix = imu::Matrix<float>::Identity();
            viewMatrix.block<3, 4>(0, 0) = Eigen::Map<const imu::ViewMatrix<float>>(pose.GetViewMatrix().val, 3, 4);

            return ToSE3(viewMatrix);
        }

        inline Pose ViewSE3ToPose(const ST::SE3<double>& se3)
        {
            cv::Matx33f viewSpaceRotationMatrix;
            cv::Vec3f viewSpacePosition;

            Eigen::Map<RotationMatrix<float>>{ viewSpaceRotationMatrix.val, 3, 3 } = Eigen::Map<const RotationMatrix<double>>{ se3.R }.cast<float>();
            Eigen::Map<Vector<float>>{ viewSpacePosition.val } = Eigen::Map<const Vector<double>>{ se3.t }.cast<float>();

            return Pose{ viewSpacePosition, viewSpaceRotationMatrix };
        }
    }
}
