// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"
#include "non_owning_stack_allocator.h"

#include <assert.h>

namespace mage
{
    namespace memory
    {
        template<typename MemAllocator, size_t Alignment = alignof(int64_t)>
        class stack_allocator : private MemAllocator, public non_owning_stack_allocator<Alignment>
        {
            using base = non_owning_stack_allocator<Alignment>;
        public:
            explicit stack_allocator(size_t size)
                : MemAllocator{}, non_owning_stack_allocator<Alignment>{ MemAllocator::allocate(size) }
            {}

            ~stack_allocator()
            {
                if (base::m_buffer != nullptr)
                {
                    MemAllocator::deallocate(base::m_buffer);
                }
            }

            using base::alignment;
            using base::adjust;
            using base::owns;
            using base::allocate;
            using base::deallocate;
        };
    }
}