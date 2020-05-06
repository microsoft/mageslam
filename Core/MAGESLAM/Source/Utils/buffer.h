// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana\utils\serialization\custom_serialization.h"
#include <assert.h>

#include <algorithm>
#include <utility>

namespace mage
{
    /*
        * IMPORTANT * MEMORY ISN'T INITIALIZED WHEN IN THIS CONTAINER
        You must initialize your objects completely/or call constructors manually.

        A buffer of typesafe uninitialized and undestroyed
        memory. This should only be used on pod/pod-like data
        that doesn't require any complex logic on destruction.
    */
    template<typename T, typename AllocT>
    struct buffer
    {
        using value_type = T;
        using pointer = value_type*;

        static_assert(std::is_trivially_destructible<T>::value, "buffer doesn't call destructors and can't be used with complex types");

        buffer(size_t size, AllocT alloc)
            : m_memory{nullptr}, m_size{0}, m_alloc{std::move(alloc)}
        {
            m_memory = std::allocator_traits<AllocT>::allocate(m_alloc, size);
            if (m_memory)
            {
                m_size = size;
            }
#ifndef NDEBUG
            fill(0xBB);
#endif
        }

        buffer(size_t size)
            : buffer(size, {})
        {}

        buffer(const buffer&) = delete;
        buffer& operator=(const buffer&) = delete;

        buffer(buffer&& other)
            : m_memory{ nullptr }, m_size{ 0 }, m_alloc{ std::move(other.m_alloc) }
        {
            std::swap(m_memory, other.m_memory);
            std::swap(m_size, other.m_size);
        }

        buffer& operator=(buffer&& other)
        {
            std::swap(m_alloc, other.m_alloc);
            std::swap(m_memory, other.m_memory);
            std::swap(m_size, other.m_size);
        }

        ~buffer()
        {
            if (m_memory)
            {
                std::allocator_traits<AllocT>::deallocate(m_alloc, m_memory, m_size);
                m_memory = nullptr; m_size = 0;
            }
        }

        void fill(int value)
        {
            memset(m_memory, value, m_size * sizeof(T));
        }

        /*
            Copies start to end into buffer at location dest.
            Returns the amount of elements copied.
        */
        size_t copy(const T* start, const T* end, size_t dest = 0)
        {
            if (start == end)
                return 0;

            assert(0 <= dest && dest < m_size);

            auto copied = std::distance(start, end);
            assert(dest + copied <= (int)m_size);
            memcpy(m_memory + dest, start, sizeof(T) * copied);
            return copied;
        }

        size_t size() const
        {
            return m_size;
        }

        T& operator[](size_t idx)
        {
            assert(0 <= idx && idx < m_size);
            return m_memory[idx];
        }

        const T& operator[](size_t idx) const
        {
            assert(0 <= idx && idx < m_size);
            return m_memory[idx];
        }

        auto data() { return m_memory; }
        auto data() const { return m_memory; }

        auto begin() { return m_memory; }
        auto end() { return m_memory + m_size; }

        auto begin() const { return m_memory; }
        auto end() const { return m_memory + m_size; }

    private:
        T* m_memory;
        size_t m_size;
        AllocT m_alloc;
    };
}

namespace mira
{
    template<typename T, typename AllocT>
    struct custom_serialization<mage::buffer<T, AllocT>>
    {
        template<typename StreamT>
        static void read(StreamT& stream, mage::buffer<T, AllocT>& buffer)
        {
            stream.read(buffer.data(), buffer.size());
        }

        template<typename StreamT>
        static void write(StreamT& stream, const mage::buffer<T, AllocT>& buffer)
        {
            stream.write(buffer.data(), buffer.size());
        }
    };
}