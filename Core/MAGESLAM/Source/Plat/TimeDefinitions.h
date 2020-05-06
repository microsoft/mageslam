// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <chrono>
#include <string>

namespace mira
{
    using HundredsNanoseconds = std::chrono::duration<int64_t, std::ratio_multiply<std::ratio<100>, std::nano>>;
    using SystemTimePoint = std::chrono::system_clock::time_point;

    using TimeUnitSecondsF = std::chrono::duration<float, std::chrono::seconds::period>;
    using TimeUnitMillisecondsF = std::chrono::duration<float, std::chrono::milliseconds::period>;
    using TimeUnitMillisecondsLL = std::chrono::duration<long long, std::chrono::milliseconds::period>;
    using TimeUnit100NanosecondsLL = std::chrono::duration<long long, std::ratio<1, 10000000>>;

    template <class Rep, class Period>
    std::wstring to_wstring(const std::chrono::duration<Rep, Period>& d)
    {
        return to_wstring(d.count());
    }
}
