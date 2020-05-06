// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once
#include <vector>
#include <set>
#include <numeric>

namespace mira
{
    class index_pool
    {
    public:
        index_pool(size_t maxSize)
            : m_maxSize{ maxSize },
            m_blockAllocationsFreeList(maxSize)
        {
            std::iota(m_blockAllocationsFreeList.begin(), m_blockAllocationsFreeList.end(), 0);
        }

        ~index_pool()
        {
            assert(available_count() == max_size() && "someone still has a reference to an item");
        }

        uint32_t acquire()
        {
            assert(m_blockAllocationsFreeList.size() > 0 && "Ran out of allocated memory for index list");
            uint32_t item = m_blockAllocationsFreeList.back();
            m_blockAllocationsFreeList.pop_back();
            m_blockAllocations.insert(item);

            return item;
        }

        void release(uint32_t item)
        {
            assert(m_blockAllocations.find(item) != m_blockAllocations.end());
            m_blockAllocations.erase(item);
            m_blockAllocationsFreeList.push_back(item);
        }

        size_t used_count() const { return m_blockAllocations.size(); }

        size_t available_count() const { return m_blockAllocationsFreeList.size(); }

        const std::set<uint32_t>& BlockAllocations() const { return m_blockAllocations; }

        size_t max_size() const { return m_maxSize; }

        void clear()
        {
            std::copy(std::begin(m_blockAllocations), std::end(m_blockAllocations), std::back_inserter(m_blockAllocationsFreeList));
            m_blockAllocations.clear();
        }

    private:
        std::vector<uint32_t> m_blockAllocationsFreeList;
        std::set<uint32_t> m_blockAllocations;

        const size_t m_maxSize;
    };
}
