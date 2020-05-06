// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "block.h"

#include <limits>

namespace mage
{
    namespace memory
    {
        /* default flag type to be able to make two std_allocator objects not the same type */
        namespace tags
        {
            struct undefined {};
        }

        /*
            Helper struct to wrap our allocator design so it conforms to the
            std allocator requirements and the std allocator_traits requirements.
        */
        template<typename T, typename AllocT, typename Tag = tags::undefined>
        class std_allocator
        {
        public:
            using value_type = T;
            using allocator_type = AllocT;

            template<typename D>
            using rebind = std_allocator<D, AllocT, Tag>;

            bool operator==(const std_allocator& other) const
            {
                return m_allocator == other.m_allocator;
            }

            bool operator!=(const std_allocator& other) const
            {
                return !(*this == other);
            }

            T* allocate(size_t n)
            {
                block blk = m_allocator->allocate(n * sizeof(T));
                return (T*)blk.memory;
            }

            void deallocate(T* pointer, size_t n)
            {
                m_allocator->deallocate({ pointer, n * sizeof(T) });
            }

            std_allocator(AllocT& allocator)
                : m_allocator{ &allocator }
            {}

            template<typename D>
            std_allocator(const std_allocator<D, AllocT, Tag>& other)
                : m_allocator{other.m_allocator}
            {}

            std_allocator(const std_allocator& allocator) = default;
            std_allocator& operator=(const std_allocator&) = default;
            std_allocator(std_allocator&& allocator) = default;
            std_allocator& operator=(std_allocator&&) = default;

        private:
            AllocT* m_allocator;

            template<typename OtherD, typename OtherAllocT, typename OtherTag>
            friend class std_allocator;
        };
    }
}
