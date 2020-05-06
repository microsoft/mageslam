// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <iterator>

namespace mage
{
    template<typename ContainerT, typename ItrT>
    struct range_type
    {
        using container_t = ContainerT;
        using iterator_t = ItrT;

        range_type(const iterator_t& beg, const iterator_t& end)
            : m_beg{ beg }, m_end{ end }
        {}

        iterator_t begin() const { return m_beg; }
        iterator_t end() const { return m_end; }

    private:
        iterator_t m_beg;
        iterator_t m_end;
    };

    template<typename C>
    auto skip_n(C&& container, size_t n)
    {
        using iter_t = decltype(std::begin(container));
        auto beg = std::begin(container);
        std::advance(beg, n);
        return range_type<std::decay_t<C>, iter_t>{beg, std::end(container)};
    }

    template<typename C>
    auto reversed(C&& container)
    {
        using iter_t = decltype(std::rbegin(container));
        return range_type<std::decay_t<C>, iter_t>{std::rbegin(container), std::rend(container)};
    }
}