// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <algorithm>
#include <iterator>
#include <vector>

#include <assert.h>

namespace mira
{
    template<typename T, typename CompareT = std::less<T>, typename AllocT = std::allocator<T>>
    struct sorted_vector
    {
        using container = std::vector<T, AllocT>;

    public:
        using iterator = typename container::iterator;
        using const_iterator = typename container::const_iterator;
        using value_type = typename container::value_type;
        using pointer = typename container::pointer;

        sorted_vector() = default;

        sorted_vector(AllocT alloc)
            : m_data{ alloc }
        {}

        sorted_vector(size_t reserve, AllocT alloc)
            : m_data{ alloc }
        {
            m_data.reserve(reserve);
        }

        sorted_vector(std::initializer_list<T> elements)
        {
            insert(elements.begin(), elements.end());
        }

        template<typename IterT>
        sorted_vector(IterT beg, IterT end)
        {
            insert(beg, end);
        }

        sorted_vector(const sorted_vector&) = default;
        sorted_vector& operator=(const sorted_vector&) = default;
        sorted_vector(sorted_vector&&) = default;
        sorted_vector& operator=(sorted_vector&&) = default;

        void reserve(size_t size)
        {
            m_data.reserve(size);
        }

        size_t size() const
        {
            return m_data.size();
        }

        bool empty() const
        {
            return m_data.empty();
        }

        template<typename CompatKeyT>
        auto find(const CompatKeyT& key) const
        {
            return find(key, [](const T& el, const CompatKeyT& key) { return el == key; });
        }

        template<typename CompatKeyT, typename EqualT>
        auto find(const CompatKeyT& key, EqualT equal) const
        {
            auto part = partition_point(key);

            if (part == m_data.end() || !equal(*part, key))
                return m_data.end();

            return part;
        }

        T& operator[](size_t idx)
        {
            return m_data[idx];
        }

        const T& operator[](size_t idx) const
        {
            return m_data[idx];
        }

        void clear()
        {
            m_data.clear();
        }

        auto begin()
        {
            return m_data.begin();
        }

        auto end()
        {
            return m_data.end();
        }

        auto begin() const
        {
            return m_data.begin();
        }

        auto end() const
        {
            return m_data.end();
        }

        auto cbegin() const
        {
            return m_data.cbegin();
        }

        auto cend() const
        {
            return m_data.cend();
        }

        auto rbegin()
        {
            return m_data.rbegin();
        }

        auto rend()
        {
            return m_data.rend();
        }

        auto rbegin() const
        {
            return m_data.rbegin();
        }

        auto rend() const
        {
            return m_data.rend();
        }

        auto& front()
        {
            return *begin();
        }

        auto& back()
        {
            return *rbegin();
        }

        auto& front() const
        {
            return *begin();
        }

        auto& back() const
        {
            return *rbegin();
        }

        auto data()
        {
            return m_data.data();
        }

        auto data() const
        {
            return m_data.data();
        }

        template<typename IterT>
        void erase(IterT beg, IterT end)
        {
            m_data.erase(beg, end);
        }

        template<typename IterT>
        void erase(IterT&& beg)
        {
            m_data.erase(std::forward<IterT>(beg));
        }

        template<typename G>
        void insert(G&& val)
        {
            m_data.insert(partition_point(val), std::forward<G>(val));
        }

        template<typename IterT>
        void insert(IterT beg, IterT end)
        {
            for (auto itr = beg; itr != end; ++itr)
            {
                insert(*itr);
            }
        }

        void insert(std::initializer_list<T> elements)
        {
            insert(elements.begin(), elements.end());
        }

        template<typename IterT>
        void insert_presorted(IterT beg, IterT end)
        {
            assert(std::is_sorted(beg, end, CompareT()));
            assert(m_data.empty());

            m_data.reserve(std::distance(beg, end));
            m_data.insert(m_data.end(), beg, end);
        }

        template<typename G>
        void insert_presorted(G&& element)
        {
            assert(m_data.empty() ? true : CompareT()(m_data.back(), element) || m_data.back() == element);
            m_data.emplace_back(std::forward<G>(element));
        }

        template<typename OtherAllocT>
        void merge(const sorted_vector<T, CompareT, OtherAllocT>& other)
        {
            auto midIdx = m_data.size();
            m_data.insert(m_data.end(), other.begin(), other.end());
            std::inplace_merge(m_data.begin(), m_data.begin() + midIdx, m_data.end(), CompareT());
        }

    private:
        template<typename CompatKeyT>
        auto partition_point(const CompatKeyT& val) const
        {
            CompareT comp{};
            return std::partition_point(
                m_data.begin(), m_data.end(), [&val, comp](const auto& existing) { return comp(existing, val); });
        }

        container m_data;
    };
}
