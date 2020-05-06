// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// Intrinsics.h
//------------------------------------------------------------------------------

#pragma once

#include <arcana/analysis/introspector.h>

#include <gsl/gsl>

#include <array>

namespace mage
{    
    class Intrinsics
    {
    public:
        Intrinsics() = default;
        Intrinsics(gsl::span<const float, 4> intrinsicsCxCyFxFy, uint32_t imageWidth, uint32_t imageHeight);
        Intrinsics(float cx, float cy, float fx, float fy, uint32_t imageWidth, uint32_t imageHeight);

        // cx, cy, fx, fy
        gsl::span<const float, 4> GetCoefficients() const { return m_coefficients; }
        std::array<float, 4> GetNormalizedCoefficients() const;

        float GetCx() const { return m_coefficients[0]; } //principle point pixels
        float GetCy() const { return m_coefficients[1]; } //principle point pixels
        float GetFx() const { return m_coefficients[2]; } //focal length pixels
        float GetFy() const { return m_coefficients[3]; } //focal length pixels

        void SetCx(float cx) { m_coefficients[0] = cx; }  //principle point pixels
        void SetCy(float cy) { m_coefficients[1] = cy; }  //principle point pixels
        void SetFx(float fx) { m_coefficients[2] = fx; }  //focal length pixels
        void SetFy(float fy) { m_coefficients[3] = fy; }  //focal length pixels

        uint32_t GetCalibrationWidth() const { return m_widthPixels; }
        uint32_t GetCalibrationHeight() const { return m_heightPixels; }

    private:
        std::array<float, 4> m_coefficients{}; // cx, cy, fx, fy in pixels
        uint32_t m_widthPixels{};  //widht/height calibration was performed for
        uint32_t m_heightPixels{};
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const Intrinsics& intrin)
    {
        intro(
            cereal::make_nvp("Coefficients", intrin.GetCoefficients())
        );
    }
}
