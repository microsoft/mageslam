// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    // helper compare object for just using a specific index in
    // a tuple/pair/array object using the std::get<int> accessor;
    template<int idx, typename T>
    struct less_idx
    {
        constexpr bool operator()(const T& lhs, const T& rhs) const
        {
            return std::get<idx>(lhs) < std::get<idx>(rhs);
        }
    };
}
