// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"
#include "passthrough_allocator.h"
#include <cstring>

namespace mage
{
    namespace memory
    {
        /*
            Allocator wrapper that invalidates memory on deallocate
        */
        template<typename AllocT>
        class debug_memory_marking_allocator : public passthrough_allocator<AllocT>
        {
            using base = passthrough_allocator<AllocT>;
        public:
            using base::passthrough_allocator;

            block allocate(size_t size)
            {
                auto blk = base::allocate(size);
#ifndef NDEBUG
                memset(blk.memory, 0xCD, blk.size);
#endif
                return blk;
            }

            void deallocate(const block& blk)
            {
#ifndef NDEBUG
                memset(blk.memory, 0xFE, blk.size);
#endif
                base::deallocate(blk);
            }
        };
    }
}