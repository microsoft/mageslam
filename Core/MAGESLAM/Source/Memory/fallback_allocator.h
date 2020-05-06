// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "allocation.h"

#include <algorithm>
#include "arcana\utils\algorithm.h"

namespace mage
{
    namespace memory
    {
        template<typename PrimaryAllocT, typename SecondaryAllocT>
        struct fallback_allocator
        {
        public:
            /*
                Allocator Interface Begins Here
            */
            static constexpr size_t alignment = mira::max(PrimaryAllocT::alignment, SecondaryAllocT::alignment);

            template<typename ...Args>
            fallback_allocator(Args&&... args)
                : m_primary{ args... }, m_secondary{}
            {}

            bool owns(void* memory)
            {
                return m_primary.owns(memory) || m_secondary.owns(memory);
            }

            block allocate(size_t size)
            {
                block aloc = m_primary.allocate(size);
                if (aloc == nullptr)
                {
                    aloc = m_secondary.allocate(size);
                }
                return aloc;
            }

            void deallocate(const block& blk)
            {
                if (m_primary.owns(blk.memory))
                    m_primary.deallocate(blk);
                else
                    m_secondary.deallocate(blk);
            }

        private:
            PrimaryAllocT m_primary;
            SecondaryAllocT m_secondary;
        };
    }
}