// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <vector>
#include <array>
#include <deque>

namespace mage
{
    /*  sliding history window of elements 
            - the most recent elements are at 0
            - the elements about to be forgotten are at N - 1
    */
    template<typename T, size_t N>
    class historical_queue
    {
    public:
        static constexpr size_t max_size = N;

        void advance(T&& newItems)
        {
            m_history.push_front(std::move(newItems));

            if (m_history.size() > N)
                m_history.pop_back();
        }

        void advance(const T& newItems)
        {
            m_history.push_front(newItems);

            if (m_history.size() > N)
                m_history.pop_back();
        }

        T& operator[](size_t i)
        {
            return m_history[i];
        }

        const T& operator[](size_t i) const
        {
            return m_history[i];
        }

        //newest Iterator
        auto begin() { return m_history.begin(); }
        auto end() { return m_history.end(); }
        //newest Iterator
        auto begin() const { return m_history.begin(); }
        auto end() const { return m_history.end(); }

        // oldest Iterator
        auto rbegin() { return m_history.rbegin(); }
        auto rend() { return m_history.rend(); }
        // oldest Iterator
        auto rbegin() const { return m_history.rbegin(); }
        auto rend() const { return m_history.rend(); }
        
        auto clear() { m_history.clear(); }

        auto& newest() { return m_history.front(); }
        auto& oldest() { return m_history.back(); }
        auto& newest() const { return m_history.front(); }
        auto& oldest() const { return m_history.back(); }

        bool full() const { return size() == max_size; }

        size_t size() const { return m_history.size(); }
    private:
        std::deque<T> m_history{};
    };
}
