// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"

#include <assert.h>

namespace mage
{
    namespace memory
    {
        template<size_t Alignment = alignof(int64_t)>
        class non_owning_stack_allocator
        {
        public:
            explicit non_owning_stack_allocator(block blk)
                : m_buffer{ blk }, m_allocated{ 0 }
            {}

            ~non_owning_stack_allocator()
            {
                m_buffer = nullptr;
            }

            void reset_to(void* memory)
            {
                assert((uintptr_t)m_buffer.memory <= (uintptr_t)memory && (uintptr_t)memory <= (uintptr_t)m_buffer.memory + m_allocated);
                m_allocated = (uintptr_t)memory - (uintptr_t)m_buffer.memory;
            }

            void* pointer() const
            {
                return (void*)((uintptr_t)m_buffer.memory + m_allocated);
            }

            /*
            Allocator Interface Begins Here
            */
            static constexpr size_t alignment = Alignment;

            size_t adjust(size_t size) const
            {
                auto remainder = size % alignment;
                if (remainder == 0)
                    return size;

                return size + (alignment - remainder);
            }

            bool owns(void* memory) const
            {
                bool inbuffer = m_buffer.memory <= memory && (uintptr_t)memory < (uintptr_t)m_buffer.memory + m_buffer.size;
                assert(inbuffer ? (uintptr_t)memory < (uintptr_t)m_buffer.memory + m_allocated : true);
                return inbuffer;
            }

            block allocate(size_t size)
            {
                size = adjust(size);

                if (m_allocated + size > m_buffer.size)
                    return{};

                void* memory = (void*)((uintptr_t)m_buffer.memory + m_allocated);
                m_allocated += size;
                return{ memory, size };
            }

            void deallocate(const block& blk)
            {
                // only deallocate if the block is the one on the end
                if ((uintptr_t)m_buffer.memory + m_allocated == (uintptr_t)blk.memory + blk.size)
                {
                    m_allocated -= blk.size;
                }
            }
        protected:
            block m_buffer;
            size_t m_allocated;
        };
    }
}