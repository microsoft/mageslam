// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "semaphore.h"
#include "signal.h"

namespace mira
{
    /*
        A tourniquet that invokes a function every n-th wait,
        and ensures the nth + 1 wait call doesn't run until
        it has.
    */
    class invoke_nth
    {
    public:
        invoke_nth(uint16_t n)
            : m_semaphore{ n }
            , m_n{ n }
        {}

        const uint16_t nth() const
        {
            return m_n;
        }

        template<typename InvokeT>
        void invoke(const cancellation& token, InvokeT&& callback)
        {
            // make sure we don't get overrun by someone lapping us
            m_semaphore.wait(token);

            auto totalReads = ++m_reads;
            if (totalReads == m_n)
            {
                // reset our state
                m_reads = 0;

                // invoke the callback
                std::invoke(std::forward<InvokeT>(callback));

                // let the next n callers through
                m_semaphore.signal(m_n);
            }
        }

    private:
        semaphore m_semaphore;
        std::atomic<uint16_t> m_reads = 0;
        const uint16_t m_n;
    };
}
