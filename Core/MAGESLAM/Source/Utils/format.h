// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <sstream>

namespace mage
{
    template<typename C>
    class basic_format
    {
    public:
        template<typename T>
        basic_format&& operator<<(T&& val) &&
        {
            m_ss << val;
            return std::move(*this);
        }

        operator std::basic_string<C, std::char_traits<C>, std::allocator<C>>() const
        {
            return m_ss.str();
        }
    private:
        std::basic_stringstream<C, std::char_traits<C>, std::allocator<C>> m_ss;
    };

    using wformat = basic_format<wchar_t>;
    using format = basic_format<char>;
}