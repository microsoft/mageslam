// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "passthrough_allocator.h"

#include <assert.h>

namespace mage
{
    namespace memory
    {
        template<typename AllocT>
        class debug_check_fail : public passthrough_allocator<AllocT>
        {
            using base = passthrough_allocator<AllocT>;
        public:
            using base::passthrough_allocator;

            block allocate(size_t size)
            {
                block blk = base::allocate(size);
                assert(blk != nullptr && "failed to allocate a block of memory");
                return blk;
            }

            void deallocate(const block& blk)
            {
                base::deallocate(blk);
            }
        };
    }
}
