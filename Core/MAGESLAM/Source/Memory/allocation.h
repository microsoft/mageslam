// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <stdint.h>

namespace mage
{
    namespace memory
    {
        template<typename T>
        struct allocation
        {
            allocation(uintptr_t location, size_t n)
                : m_location{ location }, m_size{ (location % alignof(T)) + (n * sizeof(T)) }
            {}

            allocation(T* location, size_t n)
                : allocation{ (uintptr_t)location, n }
            {}

            template<typename G>
            allocation<G> then(size_t n = 1)
            {
                return{ end(), n };
            }

            uintptr_t object() const
            {
                return aligned_location(m_location, alignof(T));
            }

            uintptr_t begin() const
            {
                return m_location;
            }

            uintptr_t end() const
            {
                return m_location + m_size;
            }

            template<typename G>
            uintptr_t aligned_end() const
            {
                return aligned_location(end(), alignof(G));
            }

        private:
            static uintptr_t aligned_location(uintptr_t location, size_t align)
            {
                return location + (location) % align;
            }

            uintptr_t m_location;
            size_t m_size;
        };
    }
}