// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <vector>
#include <array>
#include <assert.h>

#include <arcana/analysis/introspector.h>

namespace mage
{
    template<typename T>
    struct collection_traits
    {
        template<typename InputT>
        static constexpr T* convert(InputT* input)
        {
            return input;
        }
    };

    /*
        A covariant view on a buffer of contiguous objects.
        Should be used in order to pass in vector<derived> into methods
        that accept collection<base>. The lifetime of vector<derived> must
        be longer than the lifetime of collection<base>.

        A collection is always "const" in the sense that it is a view on
        a vector that can't change. You can modify the elements in a const
        collection though, because it doesn't own the elements and you
        aren't modifying the view itself.
    */
    template<typename T>
    struct collection
    {
        template<typename C>
        collection(C& vec)
            : m_begin{nullptr}, m_end{nullptr}, m_deltap{ sizeof(typename C::value_type) }
        {
            if (!vec.empty())
            {
                m_begin = collection_traits<T>::convert<typename C::value_type>(vec.data());
                m_end = collection_traits<T>::convert<typename C::value_type>(vec.data() + vec.size());
            }
        }

        template<typename D>
        collection(const collection<D>& vec)
            :   m_begin{ collection_traits<T>::convert<D>(vec.m_begin) },
                m_end{ collection_traits<T>::convert<D>(vec.m_end) },
                m_deltap{ vec.m_deltap }
        {}

        size_t size() const
        {
            return ((intptr_t)m_end - (intptr_t)m_begin) / m_deltap;
        }

        T& operator [](size_t idx) const
        {
            assert(idx >= 0 && idx < size());
            return *(T*)((intptr_t)m_begin + idx * m_deltap);
        }

        template<typename D>
        collection<D> to_collection() const
        {
            collection<D> self{ *this };
            return self;
        }

        std::vector<T> to_vector() const
        {
            std::vector<T> result;
            result.reserve(size());
            std::copy(begin(), end(), back_inserter(result));
            return result;
        }

        template<typename T>
        struct iterator : std::iterator<std::random_access_iterator_tag, T>
        {
            using value_type = T;
            using pointer_type = T*;
            using reference_type = value_type&;

            reference_type operator*() const
            {	// return designated object
                return (*m_owner)[m_idx];
            }

            reference_type operator[](ptrdiff_t offset) const
            {	// subscript
                return (*m_owner)[m_idx + offset];
            }

            pointer_type operator->() const
            {	// return pointer to class object
                return &(*m_owner)[m_idx];
            }

            iterator& operator++()
            {	// preincrement
                ++m_idx;
                return *this;
            }

            iterator operator++(int)
            {	// postincrement
                ++m_idx;
                return{ m_owner, m_idx - 1 };
            }

            iterator& operator--()
            {	// predecrement
                --m_idx;
                return *this;
            }

            iterator operator--(int)
            {	// postdecrement
                --m_idx;
                return{ m_owner, m_idx + 1 };
            }

            iterator& operator+=(ptrdiff_t offset)
            {	// increment by integer
                m_idx += offset;
                return *this;
            }

            iterator operator+(ptrdiff_t offset) const
            {	// return this + integer
                return{ m_owner, m_idx + offset };
            }

            iterator& operator-=(ptrdiff_t offset)
            {	// decrement by integer
                m_idx -= offset;
                return *this;
            }

            iterator operator-(ptrdiff_t offset) const
            {	// return this - integer
                return{ m_owner, m_idx - offset };
            }

            ptrdiff_t operator-(const iterator& right) const
            {	// return difference of iterators
                return m_idx - right.m_idx;
            }

            bool operator !=(const iterator& other) const
            {
                return m_idx != other.m_idx || m_owner != other.m_owner;
            }

            bool operator ==(const iterator& other) const
            {
                return !(*this != other);
            }

            bool operator <(const iterator& other) const
            {
                return m_idx < other.m_idx;
            }
        private:
            using container_type = std::conditional_t<std::is_const<value_type>::value, const collection, collection>;

            iterator(container_type* owner, size_t idx)
                : m_owner{ owner }, m_idx{ idx }
            {}

            container_type* m_owner;
            size_t m_idx;

            friend struct collection;
        };

        iterator<T> begin() const { return{ const_cast<collection*>(this), 0u }; }
        iterator<T> end() const { return{ const_cast<collection*>(this), size() }; }

        iterator<T> begin() { return{ this, 0u }; }
        iterator<T> end() { return{ this, size() }; }

    private:
        T* m_begin;
        T* m_end;
        ptrdiff_t m_deltap;

        template<typename D>
        friend struct collection;
    };

    template<typename ArchiveT, typename T>
    void introspect_object(mira::introspector<ArchiveT>& intro, const collection<T>& elements)
    {
        intro.callable()(cereal::SizeTag<cereal::size_type>(elements.size()));

        for (auto& el : elements)
        {
            intro.introspect(el);
        }
    }
}
