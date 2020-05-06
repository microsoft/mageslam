// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "sorted_vector.h"

#include "arcana/analysis/introspector.h"

namespace mira
{
    /*
        This structure is basically an std::set with a vector as a backing store.

        It's ideal (faster) for small (not sure what bounds yet) unique collections we want
        to iterate on afterwards.
    */
    template<typename T, typename CompareT = std::less<T>, typename AllocT = std::allocator<T>>
    struct unique_vector
    {
    public:
        unique_vector() = default;

        explicit unique_vector(AllocT alloc)
            : m_data{ std::move(alloc) }
        {}

        unique_vector(size_t reserve, AllocT alloc)
            : m_data{ std::move(alloc) }
        {
            m_data.reserve(reserve);
        }

        unique_vector(std::initializer_list<T> elements)
        {
            insert(elements.begin(), elements.end());
        }

        template<typename IterT>
        unique_vector(IterT beg, IterT end)
        {
            insert(beg, end);
        }

        unique_vector(const unique_vector&) = default;
        unique_vector& operator=(const unique_vector&) = default;
        unique_vector(unique_vector&&) = default;
        unique_vector& operator=(unique_vector&&) = default;

        void reserve(size_t size)
        {
            m_data.reserve(size);
        }

        size_t size() const
        {
            return m_data.size();
        }

        auto find(const T& element) const
        {
            auto part = partition_point(element);

            if (part == m_data.end() || *part != element)
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

        auto data()
        {
            return m_data.data();
        }

        auto data() const
        {
            return m_data.data();
        }

        template<typename G>
        bool insert(G&& val)
        {
            auto partition = partition_point(val);
            if (partition == m_data.end() || *partition != val)
            {
                m_data.insert(partition, std::forward<G>(val));
                return true;
            }
            return false;
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
            assert(std::unique(beg, end));
            assert(m_data.empty());

            m_data.reserve(std::distance(beg, end));
            m_data.insert(m_data.end(), beg, end);
        }

        template<typename G>
        void insert_presorted(G&& element)
        {
            assert(m_data.empty() ? true : CompareT()(m_data.back(), element));
            m_data.push_back(element);
        }

        template<typename OtherAllocT>
        void merge(const unique_vector<T, CompareT, OtherAllocT>& other)
        {
            std::vector<T, AllocT> merged{ m_data.get_allocator() };
            merged.reserve(size() + other.size());
            std::set_union(
                m_data.begin(), m_data.end(), other.begin(), other.end(), std::back_inserter(merged), CompareT());
            std::swap(m_data, merged);
        }

    private:
        auto partition_point(const T& val) const
        {
            CompareT comp{};
            return std::partition_point(
                m_data.begin(), m_data.end(), [&val, comp](const auto& existing) { return comp(existing, val); });
        }

        std::vector<T, AllocT> m_data;
    };

    template<typename ArchiveT, typename T, typename CompareT, typename AllocT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const unique_vector<T, CompareT, AllocT>& elements)
    {
        intro(
            gsl::make_span(elements.data(), elements.size())
        );
    }
}
