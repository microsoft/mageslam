// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <atomic>
#include <mutex>

#include "arcana\threading\cancellation.h"

namespace mira
{
    struct incrementing_counter
    {
    public:
        incrementing_counter(size_t startCount = 0)
            : m_count{ startCount }
            , m_mutex{}
            , m_signal{}
        {}

        void increment()
        {
            {
                std::unique_lock<std::mutex> lock{ m_mutex };

                m_count++;
            }

            m_signal.notify_all();
        }

        // Forces the notification of awaiters.
        // This method is usefull to avoid waiting for the
        // timeout in the case of cancellation.
        void notify()
        {
            // we need to take the lock in this case
            // and provoke a hurry up and wait because we
            // don't want to incure the timeout cost if
            // we notify right before the call to wait_for
            std::unique_lock<std::mutex> lock{ m_mutex };

            m_signal.notify_all();
        }

        void wait(size_t waitUntilCount, const cancellation& running)
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            while (waitUntilCount < m_count && !running.cancelled())
            {
                m_signal.wait_for(lock, std::chrono::milliseconds{ 10 });
            }
        }

        size_t count() const
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            return m_count;
        }

    private:
        size_t m_count;
        mutable std::mutex m_mutex;
        std::condition_variable m_signal;
    };
}