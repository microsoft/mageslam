// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/finally_scope.h"
#include "arcana/functional/inplace_function.h"
#include "arcana/utils/algorithm.h"
#include "sorted_vector.h"

#include <gsl/gsl>
#include <iterator>
#include <map>
#include <mutex>
#include <stdint.h>

namespace mira
{
    /*
        A collection that mimics a coat check. You insert something into it and it gives you a ticket.
        Once you're done with the ticket, your item gets removed from the collection.
    */
    template<typename T, typename MutexT = std::mutex>
    class ticketed_collection
    {
        using seed_t = int64_t;

        struct compare
        {
            bool operator()(const std::pair<seed_t, T>& left, const std::pair<seed_t, T>& right) const
            {
                return left.first < right.first;
            }

            bool operator()(const std::pair<seed_t, T>& left, const seed_t& right) const
            {
                return left.first < right;
            }
        };

        using storage_t = sorted_vector<std::pair<seed_t, T>, compare>;

        struct ticket_functor
        {
            ticket_functor(seed_t id, MutexT& mutex, storage_t& items)
                : m_id{ id }
                , m_mutex{ mutex }
                , m_items{ items }
            {}

            void operator()()
            {
                std::lock_guard<MutexT> guard{ m_mutex };

                auto found = m_items.find(
                    m_id, [](const std::pair<seed_t, T>& left, const seed_t& right) { return left.first == right; });

                assert(found != m_items.end() && "ticketed item wasn't found in the collection");

                m_items.erase(found);
            }

            seed_t m_id;
            MutexT& m_mutex;
            storage_t& m_items;
        };

    public:
        struct iterator
        {
            using iter_t = typename storage_t::const_iterator;

            using difference_type = typename std::iterator_traits<iter_t>::difference_type;
            using value_type = T;
            using pointer = const value_type*;
            using reference = const value_type&;
            using iterator_category = std::input_iterator_tag;

            explicit iterator(const iter_t& itr)
                : m_iter{ itr }
            {}

            bool operator==(const iterator& other) const
            {
                return m_iter == other.m_iter;
            }

            bool operator!=(const iterator& other) const
            {
                return !(*this == other);
            }

            iterator& operator++()
            {
                ++m_iter;
                return *this;
            }

            iterator operator++(int)
            {
                iterator pre{ *this };
                ++m_iter;
                return pre;
            }

            const T* operator->() const
            {
                return &m_iter->second;
            }

            const T& operator*() const
            {
                return m_iter->second;
            }

        private:
            typename storage_t::const_iterator m_iter;
        };

        using ticket = gsl::final_action<
            stdext::inplace_function<void(), sizeof(ticket_functor), alignof(ticket_functor)>>;

        using ticket_scope = finally_scope<ticket>;

        ticketed_collection() = default;
        ticketed_collection(const ticketed_collection&) = delete;
        ticketed_collection& operator=(const ticketed_collection&) = delete;

        iterator begin() const
        {
            return iterator{ m_items.cbegin() };
        }

        iterator end() const
        {
            return iterator{ m_items.cend() };
        }

        auto size() const
        {
            return m_items.size();
        }

        auto empty() const
        {
            return m_items.empty();
        }

        //
        // Inserts an element into the collection and returns a ticket
        // that will remove the item on its destruction. The mutex passed
        // in should be already locked when calling this method. That same
        // mutex will get locked when removing the item from the collection.
        //
        template<typename ElementT>
        ticket insert(ElementT&& element, MutexT& mutex)
        {
            seed_t id = m_seed++;
            m_items.insert(std::pair<seed_t, T>{ id, std::forward<ElementT>(element) });

            return ticket{ ticket_functor(id, mutex, m_items) };
        }

    private:
        seed_t m_seed = 0;
        storage_t m_items;
    };
}
