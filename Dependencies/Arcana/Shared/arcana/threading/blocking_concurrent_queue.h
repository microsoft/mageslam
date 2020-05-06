// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "cancellation.h"

#include <atomic>
#include <mutex>
#include <queue>

namespace mira
{
    template<typename T, size_t max_size = std::numeric_limits<size_t>::max()>
    class blocking_concurrent_queue
    {
        // Reasoning 1:  notify should be called outside the lock to avoid "hurry up and wait"
        // http://en.cppreference.com/w/cpp/thread/condition_variable/notify_one

    public:
        template<typename G>
        void push(G&& data)
        {
            bool notify = false;
            {
                std::unique_lock<std::mutex> lock{ m_mutex };

                notify = m_data.empty();
                m_data.push(std::forward<G>(data));

                while (m_data.size() > max_size)
                {
                    m_data.pop();
                }
            }
            if (notify)
            {
                // See Reasoning 1
                m_dataReady.notify_one();
            }
        }

        bool empty() const
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            return m_data.empty();
        }

        bool blocking_pop(T& dest, const cancellation& cancel)
        {
            return internal_pop(dest, cancel, true);
        }

        bool blocking_drain(std::vector<T>& dest, const cancellation& cancel)
        {
            return internal_drain(dest, cancel, true);
        }

        bool try_pop(T& dest, const cancellation& cancel)
        {
            return internal_pop(dest, cancel, false);
        }

        bool try_drain(std::vector<T>& dest, const cancellation& cancel)
        {
            return internal_drain(dest, cancel, false);
        }

        void clear()
        {
            std::queue<T> empty;
            {
                std::unique_lock<std::mutex> lock{ m_mutex };

                // swap with empty so that destruction of the queue items
                // don't occure in the lock
                std::swap(m_data, empty);
            }

            // See Reasoning 1
            m_dataReady.notify_all();
        }

        void cancelled()
        {
            // See Reasoning 1
            m_dataReady.notify_all();
        }

    private:
        bool internal_pop(T& dest, const cancellation& cancel, bool block)
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            if (block)
            {
                while (!cancel.cancelled() && m_data.empty())
                {
                    m_dataReady.wait(lock);
                }
            }

            if (m_data.empty() || cancel.cancelled())
                return false;

            dest = std::move(m_data.front());
            m_data.pop();

            return true;
        }

        bool internal_drain(std::vector<T>& dest, const cancellation& cancel, bool block)
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            if (block)
            {
                while (!cancel.cancelled() && m_data.empty())
                {
                    m_dataReady.wait(lock);
                }
            }

            if (m_data.empty() || cancel.cancelled())
                return false;

            while (!m_data.empty())
            {
                dest.emplace_back(std::move(m_data.front()));
                m_data.pop();
            }

            return true;
        }

        std::queue<T> m_data;
        mutable std::mutex m_mutex;
        std::condition_variable m_dataReady;
    };
}