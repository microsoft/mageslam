// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Intrinsics.h"

namespace mage
{
    Intrinsics::Intrinsics(gsl::span<const float, 4> intrinsicsCxCyFxFy, uint32_t imageWidth, uint32_t imageHeight)
        :   m_coefficients{
                intrinsicsCxCyFxFy[0],
                intrinsicsCxCyFxFy[1],
                intrinsicsCxCyFxFy[2],
                intrinsicsCxCyFxFy[3]
            },
            m_widthPixels(imageWidth),
            m_heightPixels(imageHeight)
    {}

    Intrinsics::Intrinsics(float cx, float cy, float fx, float fy, uint32_t imageWidth, uint32_t imageHeight)
        : m_coefficients{ cx, cy, fx, fy },
        m_widthPixels(imageWidth),
        m_heightPixels(imageHeight)
    {
        assert(cx / imageWidth > 0.1 && cx / imageWidth < 0.9 && "cx should be in pixel coordinates");
        assert(cy / imageHeight > 0.1 && cy / imageHeight < 0.9 && "cy should be in pixel coordinates");
    }

    std::array<float, 4> Intrinsics::GetNormalizedCoefficients() const
    {
        return {
            m_coefficients[0] / m_widthPixels,
            m_coefficients[1] / m_heightPixels,
            m_coefficients[2] / m_widthPixels,
            m_coefficients[3] / m_heightPixels
        };
    }
}
