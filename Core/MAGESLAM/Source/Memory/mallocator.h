// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"

#include <stdlib.h>

namespace mage
{
    namespace memory
    {
        class mallocator
        {
        public:
            /*
                Allocator Interface Begins Here
            */
            static constexpr size_t alignment = alignof(int64_t); //malloc should be aligned to 8

            constexpr size_t adjust(size_t size) const
            {
                return size;
            }

            /*
                there is no crossplatform way to determine if an object
                was allocated on the heap. We have to use the mallocator
                as a last resort.
            */
            bool owns(void*)
            {
                return true;
            }

            block allocate(size_t size)
            {
                void* memory = malloc(size);
                return { memory, memory != nullptr ? size : 0 };
            }

            void deallocate(const block& blk)
            {
                free(blk.memory);
            }
        };
    }
}