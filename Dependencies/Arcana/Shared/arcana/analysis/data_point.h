// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <chrono>

namespace mira
{
    template<typename ValueT>
    struct data_point
    {
        using time_point = std::chrono::system_clock::time_point;

        time_point time{};
        ValueT value{};
    };

    template<typename ValueT>
    data_point<ValueT> make_data_point(const typename data_point<ValueT>::time_point& time, ValueT value)
    {
        return { time, value };
    }
}
