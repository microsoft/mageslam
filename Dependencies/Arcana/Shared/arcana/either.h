// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <type_traits>

namespace mira
{
    template<typename FirstT, typename SecondT>
    class either
    {
        static_assert(!std::is_same<std::decay_t<FirstT>, std::decay_t<SecondT>>::value,
                      "either can only be used with distinct types");

    public:
        using first_type = FirstT;
        using second_type = SecondT;

        either(const first_type& value)
            : m_secondSet{ false }
        {
            ::new (&m_storage) first_type(value);
        }

        either(first_type&& value)
            : m_secondSet{ false }
        {
            ::new (&m_storage) first_type(std::move(value));
        }

        either(const second_type& ec)
            : m_secondSet{ true }
        {
            ::new (&m_storage) second_type(ec);
        }

        either(second_type&& ec)
            : m_secondSet{ true }
        {
            ::new (&m_storage) second_type(std::move(ec));
        }

        either(const either& other)
            : m_secondSet{ other.m_secondSet }
        {
            if (other.m_secondSet)
            {
                ::new (&m_storage) second_type(other.second());
            }
            else
            {
                ::new (&m_storage) first_type(other.first());
            }
        }

        either(either&& other)
            : m_secondSet{ other.m_secondSet }
        {
            if (other.m_secondSet)
            {
                ::new (&m_storage) second_type(std::move(other.fetch_second()));
            }
            else
            {
                ::new (&m_storage) first_type(std::move(other.fetch_first()));
            }
        }

        either& operator=(const either& other)
        {
            destroy();

            m_secondSet = other.m_secondSet;
            if (other.m_secondSet)
            {
                ::new (&m_storage) second_type(other.second());
            }
            else
            {
                ::new (&m_storage) first_type(other.first());
            }

            return *this;
        }

        either& operator=(either&& other)
        {
            destroy();

            m_secondSet = other.m_secondSet;
            if (other.m_secondSet)
            {
                ::new (&m_storage) second_type(std::move(other.fetch_second()));
            }
            else
            {
                ::new (&m_storage) first_type(std::move(other.fetch_first()));
            }

            return *this;
        }

        ~either()
        {
            destroy();
        }

        //
        // Returns the first value, throws std::runtime_error if it is not set.
        //
        first_type& first() const
        {
            if (m_secondSet)
                throw std::runtime_error("tried accessing the wrong value in an either (first)");

            return reinterpret_cast<first_type&>(m_storage);
        }

        //
        // Returns the second value, throw std::runtime_error if it is not set.
        //
        second_type& second() const
        {
            if (!m_secondSet)
                throw std::runtime_error("tried accessing the wrong value in an either (second)");

            return reinterpret_cast<second_type&>(m_storage);
        }

        //
        // Returns whether or not the first value of the ither is valid.
        //
        bool has_first() const noexcept
        {
            return !m_secondSet;
        }

        //
        // Returns whether or not the second value of the either is valid.
        //
        bool has_second() const noexcept
        {
            return !has_first();
        }

    private:
        second_type& fetch_second()
        {
            return reinterpret_cast<second_type&>(m_storage);
        }

        first_type& fetch_first()
        {
            return reinterpret_cast<first_type&>(m_storage);
        }

        void destroy()
        {
            if (m_secondSet)
            {
                fetch_second().~second_type();
            }
            else
            {
                fetch_first().~first_type();
            }
        }

        mutable std::aligned_union_t<1, first_type, second_type> m_storage;
        bool m_secondSet = false;
    };
}
