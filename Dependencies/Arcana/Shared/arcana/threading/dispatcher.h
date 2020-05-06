// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/functional/inplace_function.h"
#include "arcana/sentry.h"

#include "affinity.h"
#include "blocking_concurrent_queue.h"

#include <gsl/gsl>
#include <vector>

namespace mira
{
    template<size_t WorkSize>
    class dispatcher
    {
    public:
        using callback_t = stdext::inplace_function<void(), WorkSize>;
        static constexpr size_t work_size = WorkSize;

        template<typename T>
        void queue(T&& work)
        {
            m_work.push(std::forward<T>(work));
        }

        affinity get_affinity() const
        {
            return m_affinity;
        }

        dispatcher(const dispatcher&) = delete;
        dispatcher& operator=(const dispatcher&) = delete;

        virtual ~dispatcher() = default;

    protected:
        dispatcher() = default;
        dispatcher& operator=(dispatcher&&) = default;
        dispatcher(dispatcher&&) = default;

        bool tick(const cancellation& token)
        {
            return internal_tick(token, false);
        }

        bool blocking_tick(const cancellation& token)
        {
            return internal_tick(token, true);
        }

        /*
        Sets the dispatcher's tick thread affinity. Once this is set the methods
        on this instance will need to be called by that thread.
        */
        void set_affinity(const affinity& aff)
        {
            m_affinity = aff;
        }

        void cancelled()
        {
            m_work.cancelled();
        }

        void clear()
        {
            m_work.clear();
        }

    private:
        bool internal_tick(const cancellation& token, bool block)
        {
            GSL_CONTRACT_CHECK("thread affinity", m_affinity.check());

            if (block)
            {
                if (!m_work.blocking_drain(m_workload, token))
                    return false;
            }
            else
            {
                if (!m_work.try_drain(m_workload, token))
                    return false;
            }

            for (auto& work : m_workload)
            {
                work();
            }

            m_workload.clear();

            return true;
        }

        blocking_concurrent_queue<callback_t> m_work;
        affinity m_affinity;
        std::vector<callback_t> m_workload;
    };

    template<size_t WorkSize>
    class manual_dispatcher : public dispatcher<WorkSize>
    {
    public:
        using dispatcher<WorkSize>::blocking_tick;
        using dispatcher<WorkSize>::cancelled;
        using dispatcher<WorkSize>::clear;
        using dispatcher<WorkSize>::set_affinity;
        using dispatcher<WorkSize>::tick;
    };

    template<size_t WorkSize>
    class background_dispatcher : public dispatcher<WorkSize>
    {
    public:
        background_dispatcher()
            : m_registration{ m_cancellation.add_listener([this] { this->cancelled(); }) }
        {
            m_thread = std::thread{ [&]() {

                // TODO: Set the affinity when usage bugs are fixed.
                constexpr bool should_set_affinity{ false };
                if (should_set_affinity)
                {
                    this->set_affinity(std::this_thread::get_id());
                }

                while (!m_cancellation.cancelled())
                {
                    this->blocking_tick(m_cancellation);
                }
            } };
        }

        void cancel()
        {
            m_cancellation.cancel();

            if (m_thread.joinable())
            {
                m_thread.join();
            }

            this->clear();
        }

        ~background_dispatcher()
        {
            cancel();
        }

    private:
        std::thread m_thread;
        cancellation_source m_cancellation;
        cancellation_source::ticket m_registration;
    };
}
