// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "allocation.h"
#include "passthrough_allocator.h"

#include <assert.h>

namespace mage
{
    namespace memory
    {
        /*
            Wrapper allocator that doesn't deallocate memory, but reuses fixed
            sized blocks when we try to re-allocate.
        */
        template<typename AllocT>
        class free_list : public passthrough_allocator<AllocT>
        {
            using base = passthrough_allocator<AllocT>;

            struct node
            {
                node* next = nullptr;
            };

        public:
            template<typename ...Args>
            free_list(size_t blockSize, Args&&... args)
                :   base{ args... },
                    m_free{ nullptr },
                    m_allocated{ 0 },
                    m_blockSize{ base::adjust(blockSize) }
            {
                // we can't fit the free list in the size of the block of memory
                assert(sizeof(node) <= blockSize);
            }

            ~free_list()
            {
                assert(m_allocated == 0);

                while (m_free != nullptr)
                {
                    void* toDealloc = m_free;
                    m_free = m_free->next;
                    base::deallocate({ toDealloc, m_blockSize });
                }
            }

            /*
                Allocator Interface Begins Here
            */

            block allocate(size_t size)
            {
                assert(base::adjust(size) - size < base::alignment);

                size = base::adjust(size);

                // this allocator only supports allocating blocks of the same size
                // TODO support size with threshold
                // (ie if request is smaller than size, then allocate anyway)
                block blk;
                if (m_free == nullptr)
                {
                    blk = base::allocate(m_blockSize);
                }
                else
                {
                    node* available = m_free;
                    m_free = m_free->next;
                    blk = { available, m_blockSize };
                }

                if (blk != nullptr)
                    m_allocated++;

                return blk;
            }

            void deallocate(const block& blk)
            {
                m_allocated--;

                // TODO add max limit in order to avoid
                if (m_free == nullptr)
                {
                    m_free = static_cast<node*>(blk.memory);
                    new (m_free) node; // construct an empty node
                }
                else
                {
                    node* current = static_cast<node*>(blk.memory);
                    new (current) node; // construct an empty node

                    current->next = m_free;
                    m_free = current;
                }
            }
        private:
            node* m_free;
            size_t m_allocated;
            const size_t m_blockSize;
        };
    }
}