// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "cancellation.h"
#include "semaphore.h"

#include <memory>

namespace mira
{
    /*
        Thread fence that ensures that n-threads have reached the
        nfence.wait() call to let each one proceed.
    */
    class nfence
    {
    public:
        nfence(uint16_t n)
            : m_n{ n }
        {}

        // copyable, non-movable (can't move a mutex)
        nfence(const nfence&) = default;
        nfence& operator=(const nfence&) = default;
        nfence(nfence&&) = delete;
        nfence& operator=(nfence&&) = delete;

        /*
            Returns the number of threads that have to reach the fence.
        */
        size_t count() const
        {
            return m_n;
        }

        template<typename ResetF>
        void wait(const cancellation& token, ResetF&& resetFunc)
        {
            // this implementation is based on what is commonly called
            // a two-phase barrier. It ensures that we can't have a thread
            // lap another and have a thread run our critical section multiple
            // times.
            if (phase1(token))
            {
                std::invoke(std::forward<ResetF>(resetFunc));
            }
            phase2(token);
        }

        void wait(const cancellation& token)
        {
            wait(token, [] {});
        }

    private:
        // phase one blocks the first turnstile until all threads
        // have hit it.
        bool phase1(const cancellation& token)
        {
            bool lastThread = false;
            {
                std::unique_lock<std::mutex> guard{ m_mutex };
                m_count++;
                if (m_count == m_n)
                {
                    lastThread = true;
                    m_turnstile1.signal(m_n);
                }
            }

            m_turnstile1.wait(token);

            return lastThread;
        }

        // the second phase blocks the second turnstile until all threads
        // have had the opportunity to run the critical section between
        // these two phases. This ensures we can't have a thread lap the others
        // and bypass our synchronization.
        void phase2(const cancellation& token)
        {
            {
                std::unique_lock<std::mutex> guard{ m_mutex };
                m_count--;
                if (m_count == 0)
                    m_turnstile2.signal(m_n);
            }
            m_turnstile2.wait(token);
        }

        std::mutex m_mutex;
        semaphore m_turnstile1;
        semaphore m_turnstile2;

        uint16_t m_count = 0;
        const uint16_t m_n;
    };
}