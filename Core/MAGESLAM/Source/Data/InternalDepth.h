// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data.h"

#include <memory>

namespace mage
{
    struct InternalDepth
    {
        float NearPlaneDepth;
        float FarPlaneDepth;
        size_t SparsePointCount;
        std::unique_ptr<const ProjectedPoint[]> SparseDepth;

        InternalDepth(float nearDepth, float farDepth, size_t pointCount)
            : NearPlaneDepth{ nearDepth }, FarPlaneDepth{ farDepth }, SparsePointCount{ pointCount }
        {}

        InternalDepth()
            : NearPlaneDepth{ Depth::INVALID_DEPTH }, FarPlaneDepth{ Depth::INVALID_DEPTH }, SparsePointCount{ 0 }
        {}

        Depth AsDepth() const
        {
            return{
                NearPlaneDepth,
                FarPlaneDepth,
                { SparseDepth.get(), gsl::narrow<std::ptrdiff_t>(SparsePointCount) }
            };
        }
    };

}
