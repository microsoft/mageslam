// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <memory>

#include <assert.h>

#include "Memory\block.h"

namespace mage
{
    namespace memory
    {
        /*
            Helper class that saves a point in the allocator on construction
            then deallocates all the memory that has been allocated since
            that point on destruction.
        */
        template<typename AllocT>
        struct allocation_scope
        {
            explicit allocation_scope(AllocT& allocator)
                : m_allocator{ allocator }
            {
                m_baseline = m_allocator.pointer();

#ifndef NDEBUG
                m_oldfence = m_allocator.set_fence(m_baseline);
#endif
            }

            allocation_scope(const allocation_scope& other)
                : allocation_scope{ other.m_allocator }
            {}

            ~allocation_scope()
            {
                if (m_baseline != nullptr)
                {
                    m_allocator.reset_to(m_baseline);
                }

#ifndef NDEBUG
                m_allocator.set_fence(m_oldfence);
#endif
            }

            AllocT& allocator() const
            {
                return m_allocator;
            }
        private:
            AllocT& m_allocator;
            void* m_baseline = nullptr;

#ifndef NDEBUG
            void* m_oldfence = nullptr;
#endif
        };
    }
}