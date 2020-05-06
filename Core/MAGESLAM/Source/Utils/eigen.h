// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <Eigen/Geometry>
#include <gsl/gsl>

namespace mage
{
    inline void ToEuler(const Eigen::Quaternionf& quat, float& xRad, float& yRad, float& zRad)
    {
        Eigen::Matrix3f eigenMat = quat.toRotationMatrix();
        Eigen::Vector3f eulerAngles = eigenMat.eulerAngles(0, 1, 2);
        xRad = eulerAngles.x();
        yRad = eulerAngles.y();
        zRad = eulerAngles.z();
    }

    inline Eigen::Quaternionf FromEuler(float xRad, float yRad, float zRad)
    {
        return Eigen::AngleAxisf(xRad, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(yRad, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(zRad, Eigen::Vector3f::UnitZ());
    }

    template<int N>
    inline Eigen::Map<const Eigen::Matrix<float, N, 1>> ToCMap(gsl::span<const float, N> values)
    {
        return Eigen::Map<const Eigen::Matrix<float, N, 1>>{ values.data() };
    }

    template<int N>
    inline Eigen::Map<Eigen::Matrix<float, N, 1>> ToMap(gsl::span<float, N> values)
    {
        return Eigen::Map<const Eigen::Matrix<float, N, 1>>{ values.data() };
    }

    template<typename T>
    inline typename T::ConstMapType ToCMap(const T& element)
    {
        return T::ConstMapType{ element.data() };
    }

    template<typename T>
    inline typename T::MapType ToMap(T& element)
    {
        return T::MapType{ element.data() };
    }

}
