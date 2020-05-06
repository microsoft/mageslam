// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"
#include "passthrough_allocator.h"
#include <map>
#include <cstring>

namespace mage
{
    namespace memory
    {
        /*
        Allocator wrapper that invalidates memory on deallocate
        */
        template<typename AllocT>
        class debug_tracking_allocator : public passthrough_allocator<AllocT>
        {
            using base = passthrough_allocator<AllocT>;
        public:
            using base::passthrough_allocator;

            block allocate(size_t size)
            {
                block blk = base::allocate(size);

#ifndef NDEBUG
                size_t id = m_id++;
                auto insertion = m_allocationIds.insert({ (uintptr_t)blk.memory, id });
                assert(insertion.second && "Trying to allocate same memory block twice");
#endif

                return blk;
            }

            void deallocate(const block& blk)
            {
#ifndef NDEBUG
                auto found = m_allocationIds.find((uintptr_t)blk.memory);
                assert(found != m_allocationIds.end());
                m_allocationIds.erase((uintptr_t)blk.memory);
#endif
                base::deallocate(blk);
            }

            ~debug_tracking_allocator()
            {
                assert(m_allocationIds.empty());
            }

        private:
#ifndef NDEBUG
            size_t m_id = 0;
            std::map<uintptr_t, size_t> m_allocationIds;
#endif
        };
    }
}