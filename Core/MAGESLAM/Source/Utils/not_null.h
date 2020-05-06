// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <cstddef>
#include <assert.h>

namespace mage
{
    template<typename T>
    class not_null
    {
    public:
        not_null(std::nullptr_t) = delete;

        not_null(T* ptr)
            : m_ptr{ptr}
        {
            assert(ptr);
        }

        T* operator->()
        {
            return m_ptr;
        }

    private:
        T* m_ptr;
    };
}