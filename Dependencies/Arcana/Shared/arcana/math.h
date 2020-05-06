// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace mira
{
    //
    // Constants
    //

    template <typename T>
    static constexpr T PI = static_cast<T>(3.141592653589793238462643383279502884e+00);

    template <typename T>
    static constexpr T HALF_PI = static_cast<T>(1.570796326794896619231321691639751442e+00);

    template <typename T>
    static constexpr T E = static_cast<T>(2.718281828459045235360287471352662497e+00);

    template <typename T>
    static constexpr T CIRCLE_DEGREES = static_cast<T>(360.0);

    template <typename T>
    static constexpr T HALF_CIRCLE_DEGREES = static_cast<T>(180.0);

    template <typename T>
    static constexpr T QUARTER_CIRCLE_DEGREES = static_cast<T>(90.0);

    //
    // Scalar functions
    //

    template<typename T>
    constexpr T divide_round_up(T numerator, T denominator)
    {
        static_assert(std::is_integral<T>::value, "divide_round_up is only valid for integral types");
        static_assert(std::is_unsigned<T>::value, "divide_round_up is only valid for unsigned types");
        return (numerator + denominator - 1) / denominator;
    }

    template <typename T>
    constexpr inline T clamp(T v, T lo, T hi)
    {
        return std::min(hi, std::max(lo, v));
    }

    // Specializations for floating point types to use fmin/fmax for potentially better
    // performance and more floating point semantic correctness.

    template <typename T>
    constexpr inline T clampf(T v, T lo, T hi)
    {
        static_assert(std::is_floating_point<T>::value, "clampf should only be used with floating point types");
        return std::fmin(hi, std::fmax(lo, v));
    }

    template <>
    constexpr inline float clamp(float v, float lo, float hi)
    {
        return clampf(v, lo, hi);
    }

    template <>
    constexpr inline double clamp(double v, double lo, double hi)
    {
        return clampf(v, lo, hi);
    }

    template <>
    constexpr inline long double clamp(long double v, long double lo, long double hi)
    {
        return clampf(v, lo, hi);
    }

    //
    // Trigonometric functions
    //

    template<typename T>
    inline constexpr T deg2rad(T degrees)
    {
        static_assert(std::is_floating_point<T>::value, "trigonometry methods should only be used with floating point types");
        return degrees * (PI<T> / HALF_CIRCLE_DEGREES<T>);
    }

    template<typename T>
    inline constexpr T rad2deg(T radians)
    {
        static_assert(std::is_floating_point<T>::value, "trigonometry methods should only be used with floating point types");
        return radians * HALF_CIRCLE_DEGREES<T> / PI<T>;
    }

    template<typename T>
    inline constexpr T cubed(T x)
    {
        return x * x * x;
    }
}
