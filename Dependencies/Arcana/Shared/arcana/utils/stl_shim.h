// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <cstddef>

namespace std
{
    // TODO: remove this once we update the toolchain (std::apply is a c++17 library feature)
    namespace detail
    {
        template<class F, class Tuple, std::size_t... I>
        constexpr decltype(auto) apply_impl(F&& f, Tuple&& t, std::index_sequence<I...>)
        {
            return std::invoke(std::forward<F>(f), std::get<I>(std::forward<Tuple>(t))...);
        }
    } // namespace detail

    template<class F, class Tuple>
    constexpr decltype(auto) apply(F&& f, Tuple&& t)
    {
        return detail::apply_impl(std::forward<F>(f),
                                  std::forward<Tuple>(t),
                                  std::make_index_sequence<std::tuple_size_v<std::decay_t<Tuple>>>{});
    }
}