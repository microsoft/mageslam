// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <future>

namespace mira
{
    class cancellation;

    class base_signal
    {
    public:
        // not copyable, but movable
        base_signal(const base_signal&) = delete;
        base_signal& operator=(const base_signal&) = delete;

        void set()
        {
            m_signal.set_value();
        }

    protected:
        base_signal() = default;
        base_signal(base_signal&&) = default;
        base_signal& operator=(base_signal&&) = default;

        std::promise<void> m_signal;
    };

    template<typename FutureT>
    class basic_signal : public base_signal
    {
    public:
        basic_signal()
            : m_wait{ m_signal.get_future() }
        {}

        // not copyable, but movable
        basic_signal(const basic_signal&) = delete;
        basic_signal& operator=(const basic_signal&) = delete;
        basic_signal(basic_signal&&) = default;
        basic_signal& operator=(basic_signal&&) = default;

        void reset()
        {
            m_signal = std::promise<void>{};
            m_wait = m_signal.get_future();
        }

        void wait(const cancellation& token, bool autoreset = true);

    private:
        FutureT m_wait;
    };

    extern template class basic_signal<std::future<void>>;
    extern template class basic_signal<std::shared_future<void>>;

    using signal = basic_signal<std::future<void>>;
    using shared_signal = basic_signal<std::shared_future<void>>;
}