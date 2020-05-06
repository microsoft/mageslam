// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <chrono>

namespace mira
{
    struct timer
    {
        using duration_ms = std::chrono::duration<float, std::chrono::milliseconds::period>;
        using duration_s = std::chrono::duration<float, std::chrono::seconds::period>;
        using clock = std::chrono::high_resolution_clock;

        struct scope_time
        {
            scope_time(timer& timer)
                : m_timer{ timer }
            {
                m_start = m_clock.now();
            }

            ~scope_time()
            {
                m_timer.add_sample(m_clock.now() - m_start);
            }

        private:
            timer& m_timer;
            clock m_clock;
            clock::time_point m_start;
        };

        scope_time time_scope()
        {
            return { *this };
        }

        duration_ms average_ms() const
        {
            return { m_samples == 0 ? duration_ms{ 0 } : m_total / (float)m_samples };
        }

        duration_ms last_ms() const
        {
            return m_samples == 0 ? duration_ms{ 0 } : duration_ms{ m_last };
        }

        float average_fps() const
        {
            duration_s seconds = m_total;
            return seconds.count() <= 0 ? 0 : m_samples / seconds.count();
        }

        float instant_fps() const
        {
            duration_s seconds = m_last;
            return seconds.count() <= 0 ? 0 : 1.0f / seconds.count();
        }

        void add_sample(const std::chrono::nanoseconds& ns)
        {
            m_last = ns;
            m_total += ns;
            if (m_samples < AVERAGE_SAMPLE_COUNT)
            {
                m_samples++;
            }
            m_total -= m_buffer[m_bufferIndex];
            m_buffer[m_bufferIndex] = ns;
            m_bufferIndex++;
            m_bufferIndex = m_bufferIndex % AVERAGE_SAMPLE_COUNT;
        }

    private:
        static constexpr int AVERAGE_SAMPLE_COUNT = 30;

        std::chrono::nanoseconds m_total{ 0 };
        std::chrono::nanoseconds m_last{ 0 };
        std::array<std::chrono::nanoseconds, AVERAGE_SAMPLE_COUNT> m_buffer{};
        int m_samples = 0;
        int m_bufferIndex = 0;
    };

    struct tick_timer
    {
        using clock = std::chrono::high_resolution_clock;

        void tick()
        {
            if (m_initialized)
            {
                timer.add_sample(m_clock.now() - m_start);
            }
            else
            {
                m_initialized = true;
            }
            m_start = m_clock.now();
        }

        timer timer;

    private:
        clock m_clock;
        clock::time_point m_start;
        bool m_initialized = false;
    };
}