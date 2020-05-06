// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "passthrough_allocator.h"

namespace mage
{
    namespace memory
    {
        template<typename AllocT>
        class non_deallocating_allocator : public passthrough_allocator<AllocT>
        {
            using base = passthrough_allocator<AllocT>;
        public:
            using base::passthrough_allocator;

            block allocate(size_t size)
            {
                return base::allocate(size);
            }

            void deallocate(const block&)
            {
                // do nothing
            }
        };
    }
}