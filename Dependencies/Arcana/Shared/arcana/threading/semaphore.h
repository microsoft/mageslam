// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana\threading\cancellation.h"

#include <mutex>

namespace mira
{
    class semaphore
    {
    public:
        semaphore(uint16_t n = 0)
            : m_n{ n }
        {}

        void signal(uint16_t n = 1)
        {
            std::unique_lock<std::mutex> guard{ m_mutex };

            m_n += n;

            for (size_t i = 0; i < n; ++i)
            {
                m_signal.notify_one();
            }
        }

        void wait(const cancellation& token)
        {
            std::unique_lock<std::mutex> guard{ m_mutex };

            while (!token.cancelled() && m_n == 0)
            {
                m_signal.wait_for(guard, std::chrono::milliseconds{ 10 });
            }

            // If the operation gets cancelled threads are going to
            // get out of the while loop even when m_n is 0. We have to
            // make sure to decrement in the normal case, but not put
            // the semaphore in a bad state in the cancellation case.
            m_n = std::max(0, m_n - 1);
        }

    private:
        std::mutex m_mutex;
        std::condition_variable m_signal;

        int m_n = 0;
    };
}