// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"

namespace mage
{
    namespace memory
    {
        /*
            Helper public base class for passthrough allocators so they don't
            have to redefine everything.
        */
        template<typename AllocT>
        class passthrough_allocator : private AllocT
        {
        public:
            using allocator = AllocT;

            using AllocT::AllocT;

            /*
                Allocator Interface Begins Here
            */
            static constexpr size_t alignment = AllocT::alignment;

            constexpr size_t adjust(size_t size) const
            {
                return AllocT::adjust(size);
            }

            bool owns(void* memory)
            {
                return AllocT::owns(memory);
            }

            void* pointer() const
            {
                return AllocT::pointer();
            }

            void reset_to(void* dest)
            {
                AllocT::reset_to(dest);
            }

            /* implementations only need to implement these functions
                
            block allocate(size_t size);
            void deallocate(const block& blk);

            */

        protected:
            block allocate(size_t size)
            {
                return AllocT::allocate(size);
            }

            void deallocate(const block& blk)
            {
                AllocT::deallocate(blk);
            }
        };
    }
}