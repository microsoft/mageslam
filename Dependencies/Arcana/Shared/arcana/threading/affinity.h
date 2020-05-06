// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <thread>

namespace mira
{
    class affinity
    {
    public:
        affinity(const std::thread::id& id)
            : m_thread{ id }
            , m_set{ true }
        {}

        affinity()
            : m_set{ false }
        {}

        bool check() const
        {
            if (!m_set)
                return true;

            return std::this_thread::get_id() == m_thread;
        }

        bool is_set() const
        {
            return m_set;
        }

    private:
        std::thread::id m_thread;
        bool m_set; // TODO 12333940: replace with std::optional when updating to C++17
    };
}
