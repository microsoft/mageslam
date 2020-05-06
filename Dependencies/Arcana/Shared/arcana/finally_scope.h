// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <vector>

namespace mira
{
    template<typename T>
    class finally_scope
    {
    public:
        finally_scope& operator+=(T&& element)
        {
            m_elements.emplace_back(std::move(element));
            return *this;
        }

        finally_scope() = default;

        finally_scope(const finally_scope&) = delete;
        finally_scope(finally_scope&&) = default;

        finally_scope& operator=(const finally_scope&) = delete;
        finally_scope& operator=(finally_scope&&) = default;

        void clear()
        {
            m_elements.clear();
        }

    private:
        std::vector<T> m_elements;
    };
}