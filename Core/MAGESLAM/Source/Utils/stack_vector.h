// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <malloc.h>
#include <assert.h>

namespace mage
{
    template<typename T>
    struct stack_vector
    {
        using value_type = T;
        using allocator_type = void;
        using size_type = std::size_t;
        using difference_type = std::ptrdiff_t;
        using reference = value_type&;
        using const_reference = const value_type&;
        using pointer = value_type*;
        using const_pointer = const value_type*;
        using iterator = pointer;
        using const_iterator = const_pointer;
        using reverse_iterator = pointer;
        using const_reverse_iterator = const_pointer;

        stack_vector(size_t count, void* buffer)
            : m_capacity{ count }, m_count{ 0 }, m_buffer{(T*)buffer}
        {}

        stack_vector(const stack_vector&) = delete;
        stack_vector& operator=(const stack_vector&) = delete;

        size_t size() const
        {
            return m_count;
        }

        bool empty() const
        {
            return m_count == 0;
        }

        size_t capacity() const
        {
            return m_capacity;
        }

        size_t max_size() const
        {
            return m_capacity;
        }

        const T& at(size_t idx) const
        {
            assert(idx >= 0 && idx < size());
            return m_buffer[idx];
        }

        const T& operator [](size_t idx) const
        {
            assert(idx >= 0 && idx < size());
            return m_buffer[idx];
        }

        const T& front() const
        {
            return at(0);
        }

        const T& back() const
        {
            return at(size() - 1);
        }

        const T* data() const
        {
            return m_buffer;
        }

        void push_back(const T& value)
        {
            assert(m_count < m_capacity);

            *(m_buffer + m_count) = value;
            m_count++;
        }

        template<typename ...Args>
        void emplace_back(Args... args)
        {
            assert(m_count < m_capacity);

            (m_buffer + m_count)->T(std::forward<Args>(args)...);
            m_count++;
        }

        auto begin() const { return m_buffer; }
        auto end() const { return m_buffer + size(); }
        auto begin() { return m_buffer; }
        auto end() { return m_buffer + size(); }

        auto rbegin() const { return m_buffer + size() - 1; }
        auto rend() const { return m_buffer - 1; }
        auto rbegin() { return m_buffer + size() - 1; }
        auto rend() { return m_buffer - 1; }

        auto cbegin() const { return begin(); }
        auto cend() const { return end(); }
        auto crbegin() const { return rbegin(); }
        auto crend() const { return rend(); }
    private:
        size_t m_capacity;
        size_t m_count;
        T* m_buffer;
    };
}

#define init_stack_vector(T, count) count, alloca(count * sizeof(T))