// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "either.h"

#include <assert.h>
#include <system_error>

namespace mira
{
    template<typename T>
    class expected
    {
        using uninitialized_t = std::integer_sequence<int, 0>;

    public:
        using value_type = T;
        using error_type = std::error_code;

        expected()
            : expected(uninitialized_t{})
        {}

        expected(const value_type& value)
            : m_data{ value }
        {}

        expected(value_type&& value)
            : m_data{ std::move(value) }
        {}
        
        template<typename ErrorT, typename =
            std::enable_if_t<
                std::is_error_code_enum<std::decay_t<ErrorT>>::value ||
                std::is_error_condition_enum<std::decay_t<ErrorT>>::value>>
        expected(ErrorT errorEnum)
            : m_data{ std::make_error_code(errorEnum) }
        {}

        expected(const error_type& ec)
            : m_data{ ec }
        {
            assert(ec && "you should never build an expected<T> with a non-error");
        }

        expected(error_type&& ec)
            : m_data{ std::move(ec) }
        {
            assert(ec && "you should never build an expected<T> with a non-error");
        }

        //
        // Returns the expected value, throws std::runtime_error if it is in an error state.
        //
        value_type& value() const
        {
            return m_data.first();
        }

        //
        // Returns the value contained or the default passed in if the expected is in an error state.
        //
        template<typename U>
        value_type value_or(U&& def) const
        {
            if (!m_data.has_first())
                return std::forward<U>(def);

            return m_data.first();
        }

        //
        // Returns the error, throw std::runtime_error if it is not in an error state
        //
        const error_type& error() const
        {
            return m_data.second();
        }

        //
        // Returns whether or not the expected contains a value.
        //
        bool has_value() const noexcept
        {
            return m_data.has_first();
        }

        //
        // Returns whether or not the expected is in an error state.
        //
        bool has_error() const noexcept
        {
            return m_data.has_second();
        }

        //
        // Converts to true if the value is valid.
        //
        explicit operator bool() const noexcept
        {
            return m_data.has_first();
        }

        //
        // Shorthand access operators
        //
        value_type& operator*() const
        {
            return value();
        }

        //
        // Shorthand access operators
        //
        value_type* operator->() const
        {
            return &value();
        }

    private:
        expected(uninitialized_t)
            : m_data{ std::make_error_code(std::errc::broken_pipe) }
        {}

        either<value_type, error_type> m_data;
    };

    template<>
    class expected<void>
    {
    public:
        using value_type = void;
        using error_type = std::error_code;

        expected()
            : m_error{}
        {}

        template<typename ErrorT, typename =
            std::enable_if_t<
                std::is_error_code_enum<std::decay_t<ErrorT>>::value ||
                std::is_error_condition_enum<std::decay_t<ErrorT>>::value>>
        expected(ErrorT errorEnum)
            : m_error{ std::make_error_code(errorEnum) }
        {
            assert(m_error && "you should never build an expected<void> with a non-error");
        }

        expected(const error_type& ec)
            : m_error{ ec }
        {
            assert(m_error && "you should never build an expected<void> with a non-error");
        }

        expected(error_type&& ec)
            : m_error{ std::move(ec) }
        {
            assert(m_error && "you should never build an expected<void> with a non-error");
        }

        expected(const expected& other) = default;
        expected(expected&& other) = default;
        expected& operator=(const expected& other) = default;
        expected& operator=(expected&& other) = default;

        //
        // Returns the error, throw std::runtime_error if it is not in an error state
        //
        const error_type& error() const
        {
            if (!has_error())
                throw std::runtime_error("tried accessing the error on a valid expected");

            return m_error;
        }

        //
        // Returns whether or not the expected is in an error state.
        //
        bool has_error() const noexcept
        {
            return static_cast<bool>(m_error);
        }

        //
        // Converts to true if the value is valid.
        //
        explicit operator bool() const noexcept
        {
            return !has_error();
        }

        //
        // Creates an expected<void> that doesn't have an error
        //
        static expected<void> make_valid()
        {
            return expected<void>{};
        }

    private:
        error_type m_error;
    };

    template<typename ResultT>
    struct as_expected
    {
        using type = expected<ResultT>;
        using result = ResultT;
    };

    template<typename ResultT>
    struct as_expected<expected<ResultT>>
    {
        using type = expected<ResultT>;
        using result = ResultT;
    };
}
