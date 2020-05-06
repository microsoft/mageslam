// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <tuple>

namespace mira
{
    template<typename ... InterfaceTs>
    struct mixin_ptr
    {
        mixin_ptr() = default;

        template<typename T>
        mixin_ptr(T* object)
            : m_ptrs{ static_cast<InterfaceTs*>(object)... }
        {}

        operator bool() const
        {
            return std::get<0>(m_ptr) != nullptr;
        }

        template<typename T>
        T* get() const
        {
            return std::get<T*>(m_ptrs);
        }

    private:
        std::tuple<InterfaceTs*...> m_ptrs{};
    };
}
