// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "signal.h"

#include "arcana/containers/ticketed_collection.h"

#include <algorithm>
#include <assert.h>
#include <atomic>
#include <memory>
#include <vector>

#include <functional>
#include <vector>

namespace mira
{
    class cancellation
    {
        using collection = ticketed_collection<std::function<void>>;

    public:
        using ticket = collection::ticket;
        using ticket_scope = collection::ticket_scope;

        using ptr = std::shared_ptr<cancellation>;

        virtual bool cancelled() const = 0;

        /*
            Adds a callback that will get called on cancellation. You cancellation
            will get called synchronously if cancellation has already happened.
        */
        template<typename CallableT>
        ticket add_listener(CallableT&& callback)
        {
            if (this == &none())
                return ticket{ [] {} };

            std::function<void()> copied;
            ticket result{ internal_add_listener(callback, copied) };

            if (copied)
                copied();

            return result;
        }

        /*
            Waits for the cancellation token to get reset
        */
        void wait_for_reset(const cancellation& cancel)
        {
            if (this == &none())
                return;

            m_resetSignal.wait(cancel, true /* auto reset */);
        }

        static cancellation& none();

    protected:
        cancellation() = default;
        cancellation& operator=(const cancellation&) = default;

        ~cancellation()
        {
            assert(m_listeners.empty() && "you're destroying the listener collection and you still have listeners");
        }

        template<typename CallableT>
        ticket internal_add_listener(CallableT&& callback, std::function<void()>& copied)
        {
            std::lock_guard<std::mutex> guard{ m_mutex };

            if (m_signaled)
            {
                copied = std::forward<CallableT>(callback);
                return m_listeners.insert(copied, m_mutex);
            }
            else
            {
                return m_listeners.insert(std::forward<CallableT>(callback), m_mutex);
            }
        }

        void signal_cancelled()
        {
            std::vector<std::function<void()>> listeners;

            {
                std::lock_guard<std::mutex> guard{ m_mutex };

                listeners.reserve(m_listeners.size());
                std::copy(m_listeners.begin(), m_listeners.end(), std::back_inserter(listeners));

                m_signaled = true;
            }

            // We want to signal cancellation in reverse order
            // so that if a parent function adds a listener
            // then a child function does the same, the child
            // cancellation runs first. This avoids ownership
            // semantic issues.
            for(auto itr = listeners.rbegin(); itr != listeners.rend(); ++itr)
                (*itr)();
        }

        void signal_reset()
        {
            m_signaled = false;
            m_resetSignal.set();
        }

    private:
        mutable std::mutex m_mutex;
        ticketed_collection<std::function<void()>> m_listeners;
        signal m_resetSignal;
        bool m_signaled = false;
    };

    class cancellation_source : public cancellation
    {
    public:
        using ptr = std::shared_ptr<cancellation_source>;

        virtual bool cancelled() const override
        {
            return m_cancellationRequested;
        }

        void reset()
        {
            m_cancellationRequested = false;

            signal_reset();
        }

        void cancel()
        {
            if (m_cancellationRequested.exchange(true) == false)
            {
                signal_cancelled();
            }
        }

    private:
        std::atomic<bool> m_cancellationRequested{ false };
    };

    inline cancellation& cancellation::none()
    {
        static cancellation_source n{};
        return n;
    }
}
