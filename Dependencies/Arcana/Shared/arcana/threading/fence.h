// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "signal.h"

#include <assert.h>
#include <memory>

namespace mira
{
    class fence
    {
    public:
        fence() = default;

        // non copyable
        fence(const fence&) = delete;
        fence& operator=(const fence&) = delete;

        // movable
        fence(fence&&) = default;
        fence& operator=(fence&&) = default;

        void wait(const cancellation& token)
        {
            assert(m_first && m_second && "fence must be paired before being waited on");

            m_first->set();
            m_second->wait(token, true /* auto reset */);
        }

        fence make_pair()
        {
            assert(!m_owner && !m_first && !m_second && "a fence can only be paired once");
            m_owner = std::make_unique<state>();
            m_first = &m_owner->first;
            m_second = &m_owner->second;

            return { m_second, m_first };
        }

    private:
        struct state
        {
            signal first;
            signal second;
        };

        fence(signal* first, signal* second)
            : m_first{ first }
            , m_second{ second }
        {}

        std::unique_ptr<state> m_owner;

        signal* m_first = nullptr;
        signal* m_second = nullptr;
    };
}