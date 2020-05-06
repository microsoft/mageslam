// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    template<typename T = void>
    class sentry
    {
    public:
        class guard
        {
        public:
            ~guard()
            {
                if (m_sentry)
                {
                    m_sentry->m_value = m_previous;
                }
            }

            guard(guard&& other)
                : m_previous{ other.m_previous }
                , m_sentry{ other.m_sentry }
            {
                other.m_sentry = nullptr;
            }

        private:
            bool m_previous;
            sentry* m_sentry;

            guard(sentry* sen)
                : m_previous{ sen->m_value }
                , m_sentry{ sen }
            {
                sen->m_value = true;
            }

            friend class sentry;
        };

        guard take()
        {
            return { this };
        }

        bool is_active() const
        {
            return m_value;
        }

    private:
        bool m_value = false;
    };
}