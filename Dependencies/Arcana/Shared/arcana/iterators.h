// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <iterator>
#include <tuple>
#include <utility>

namespace mira
{
    template<typename C>
    struct pointer_inserter_iterator : std::iterator<std::output_iterator_tag, void, void, void, void>
    {
        using _Unchecked_type = pointer_inserter_iterator;

        pointer_inserter_iterator& operator=(std::remove_pointer_t<typename C::value_type>& value)
        {
            results->push_back(&value);
            return *this;
        }

        pointer_inserter_iterator& operator*()
        {
            return *this;
        }

        pointer_inserter_iterator& operator++()
        {
            return *this;
        }

        pointer_inserter_iterator operator++(int)
        {
            return *this;
        }

        explicit pointer_inserter_iterator(C& output)
            : results{ &output }
        {}

        C* results;
    };

    template<typename C>
    pointer_inserter_iterator<C> pointer_inserter(C& container)
    {
        return pointer_inserter_iterator<C>{ container };
    }

    template<typename C>
    struct emplace_inserter_iterator : std::iterator<std::output_iterator_tag, void, void, void, void>
    {
        using _Unchecked_type = emplace_inserter_iterator;

        emplace_inserter_iterator(const emplace_inserter_iterator&) = default;
        emplace_inserter_iterator(emplace_inserter_iterator&&) = default;
        emplace_inserter_iterator& operator=(const emplace_inserter_iterator&) = default;
        emplace_inserter_iterator& operator=(emplace_inserter_iterator&&) = default;

        template<typename T,
                 typename = std::enable_if_t<!std::is_same<emplace_inserter_iterator, std::decay_t<T>>::value>>
        emplace_inserter_iterator& operator=(T&& value)
        {
            results->emplace_back(std::forward<T>(value));
            return *this;
        }

        emplace_inserter_iterator& operator*()
        {
            return *this;
        }

        emplace_inserter_iterator& operator++()
        {
            return *this;
        }

        emplace_inserter_iterator operator++(int)
        {
            return *this;
        }

        explicit emplace_inserter_iterator(C& output)
            : results{ &output }
        {}

        C* results;
    };

    template<typename C>
    emplace_inserter_iterator<C> emplace_inserter(C& container)
    {
        return emplace_inserter_iterator<C>{ container };
    }

    template<typename TupleT>
    struct tuple_iterator
    {
        using tuple_t = TupleT;
        using indexing = std::make_index_sequence<std::tuple_size<tuple_t>::value>;

        template<typename ToIterate, typename IteratorT>
        static void iterate(ToIterate&& tuple, IteratorT&& iterator)
        {
            iterate(std::forward<ToIterate>(tuple), std::forward<IteratorT>(iterator), indexing{});
        }

    private:
        template<typename ToIterate, typename IteratorT, size_t... Index>
        static void iterate(ToIterate&& tuple, IteratorT&& iterator, std::integer_sequence<size_t, Index...>)
        {
            int unused[] = { (iterator(std::get<Index>(tuple), std::integral_constant<int, Index>{}), 0)... };
            (void)unused;
        }
    };

    template<typename TupleT, typename IteratorT>
    void iterate_tuple(TupleT&& tuple, IteratorT&& iterator)
    {
        tuple_iterator<std::decay_t<TupleT>>::iterate(std::forward<TupleT>(tuple), std::forward<IteratorT>(iterator));
    }

    template<size_t N>
    struct static_for_iterator
    {
        using indexing = std::make_index_sequence<N>;

        template<typename IteratorT>
        static void iterate(IteratorT&& iterator)
        {
            iterate(std::forward<IteratorT>(iterator), indexing{});
        }

    private:
        template<typename IteratorT, size_t... Index>
        static void iterate(IteratorT&& iterator, std::integer_sequence<size_t, Index...>)
        {
            int unused[] = { (iterator(Index), 0)... };
            (void)unused;
        }
    };

    template<size_t N, typename IteratorT>
    void static_for(IteratorT&& iterator)
    {
        static_for_iterator<N>::iterate(std::forward<IteratorT>(iterator));
    }

    template<typename IteratorT, typename ...ArgTs>
    void static_foreach(IteratorT&& iterator, ArgTs&& ...args)
    {
        int unused[] = { (iterator(args), 0)..., 0 };
        (void)iterator; // in case of empty set
        (void)unused;
    }
}
