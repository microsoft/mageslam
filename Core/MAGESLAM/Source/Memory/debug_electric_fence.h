// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"
#include "passthrough_allocator.h"
#include <assert.h>

namespace mage
{
    namespace memory
    {
        /*
            Allocator wrapper that makes sure we don't deallocate memory that we've protected
        */
        template<typename AllocT>
        class debug_electric_fence : public passthrough_allocator<AllocT>
        {
            using base = passthrough_allocator<AllocT>;
        public:
            using base::passthrough_allocator;

            void* set_fence(void* memory)
            {
                auto old = m_fence;
                m_fence = memory;
                return old;
            }

            block allocate(size_t size)
            {
                return base::allocate(size);
            }

            void deallocate(const block& blk)
            {
                assert((uintptr_t)m_fence <= (uintptr_t)blk.memory && "tried to deallocate memory that was behind our electric fence");

                base::deallocate(blk);
            }

        private:
            void* m_fence = nullptr;
        };
    }
}