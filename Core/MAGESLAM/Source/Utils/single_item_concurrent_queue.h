// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <mutex>

#include <arcana\threading\cancellation.h>

namespace mage
{
    template<typename T>
    class single_item_concurrent_queue
    {
        // Reasoning 1:  notify should be called outside the lock to avoid "hurry up and wait" http://en.cppreference.com/w/cpp/thread/condition_variable/notify_one
        // Reasoning 2:  T destructors should be called outside the lock.  They may acquire another lock, be reentrant, etc.

    public:
        void push(std::unique_ptr<T>&& data)
        {
            bool notify = false;
            std::unique_ptr<T> previousData; // See Reasoning 2

            {
                std::unique_lock<std::mutex> lock{ m_mutex };

                notify = m_data == nullptr;
                previousData = std::move(m_data);
                m_data = std::move(data);
            }

            if (notify)
            {
                // See Reasoning 1
                m_dataReady.notify_one();
            }
        }

        std::unique_ptr<T> blocking_pop(const mira::cancellation& cancel)
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            while (!cancel.cancelled() && !m_data)
            {
                m_dataReady.wait(lock);
            }
            
            return std::move(m_data);
        }

        void cancelled()
        {
            // See Reasoning 1
            m_dataReady.notify_all();
        }

        void clear()
        {
            std::unique_lock<std::mutex> lock{ m_mutex };

            m_data = nullptr;

            m_dataReady.notify_all();
        }

    private:
        std::unique_ptr<T> m_data;

        std::mutex m_mutex;
        std::condition_variable m_dataReady;
    };
}