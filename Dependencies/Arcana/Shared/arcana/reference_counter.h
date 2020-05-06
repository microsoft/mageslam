// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <atomic>
#include <mutex>

namespace mira
{
    /*
    Simple class to which keeps track of the maximum number of references to it
    */
    class reference_counter final
    {
    public:
        reference_counter() : m_refCount(0), m_maxCount(0)
        {}

        void add_reference()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (++m_refCount > m_maxCount)
            {
                m_maxCount = m_refCount;
            }
        }

        void remove_reference()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_refCount--;
        }

        float percent_of_max() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_maxCount == 0)
            {
                return 1.0f;
            }
            return static_cast<float>(m_refCount) / m_maxCount;
        }

        uint32_t current_count() const
        {
            return m_refCount;
        }

    private:
        mutable std::mutex m_mutex;
        uint32_t m_refCount;
        uint32_t m_maxCount;
    };

    /*
    Class which creates a reference to a "reference_counter" class for its lifetime
    */
    class scoped_reference final
    {
    public:
        explicit scoped_reference(reference_counter& counter) : m_counter(&counter)
        {
            m_counter->add_reference();
        }

        scoped_reference(const scoped_reference& rhs) : m_counter(rhs.m_counter)
        {
            if (m_counter != nullptr)
            {
                m_counter->add_reference();
            }
        }

        scoped_reference& operator=(const scoped_reference& rhs)
        {
            if (m_counter != nullptr)
            {
                m_counter->remove_reference();
            }
            m_counter = rhs.m_counter;
            if (m_counter != nullptr)
            {
                m_counter->add_reference();
            }
            return *this;
        }

        scoped_reference(scoped_reference&& rhs) : m_counter(rhs.m_counter)
        {
            rhs.m_counter = nullptr;
        }

        scoped_reference& operator=(scoped_reference&& rhs)
        {
            if (m_counter != nullptr)
            {
                m_counter->remove_reference();
            }
            m_counter = rhs.m_counter;
            rhs.m_counter = nullptr;
            return *this;
        }

        ~scoped_reference()
        {
            if (m_counter != nullptr)
            {
                m_counter->remove_reference();
            }
        }

    private:
        reference_counter* m_counter;
    };
}
