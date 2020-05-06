// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <chrono>

namespace mage
{
    using hundred_nanoseconds = std::chrono::duration<int64_t, std::ratio_multiply<std::ratio<100>, std::nano>>;

    template<typename T = int64_t>
    using hundred_nanoseconds_t = std::chrono::duration<T, std::ratio_multiply<std::ratio<100>, std::nano>>;
}
