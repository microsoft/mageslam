// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <future>

namespace mira
{
    // This should only be used for test tools or other rare situations where it's acceptable to use all available CPU power, including
    // blocking the current thread with a portion of the scheduled work.
    // Most situations should use async tasks instead.
    template<typename LambdaT>
    void parallel(size_t count, size_t threads, LambdaT&& lambda)
    {
        using RetT = decltype(lambda(0));

        const size_t div = count / threads;

        // Make sure to use the current thread context even when the job count is perfectly
        // divisible by the thread count.
        if (threads > 0 &&
            count % threads == 0)
        {
            threads--;
        }

        std::vector<std::thread> computes;
        computes.reserve(threads);

        for (size_t i = 0; i < threads; ++i)
        {
            // TODO: Switch this back to using std::async and std::futures once std::async regression bug is fixed.
            computes.push_back(std::thread([lambda, i, div]() mutable {
                for (size_t k = 0; k < div; ++k)
                {
                    lambda(i * div + k);
                }
            }));
        }

        for (size_t k = threads * div; k < count; ++k)
        {
            lambda(k);
        }

        for (auto& work : computes)
        {
            work.join();
        }
    }
}
