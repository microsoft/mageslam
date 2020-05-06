// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

/*
    Helper functions for asserting based on the date.

    If we were to want a piece of code to be removed before a certain date
    we would add this statement:

    REMOVE_BEFORE(Jun, 9, "This code should be removed");
*/

namespace mage
{
    namespace compile_time
    {
        template<int Size>
        constexpr bool string_startswith(const char(&prefix)[Size], const char* text, int idx = 0)
        {
            return idx == Size - 1 ? true :
                (prefix[idx] != text[idx] ? false : string_startswith(prefix, text, idx + 1));
        }

        template<int Size>
        constexpr bool this_month(const char(&month)[Size])
        {
            return string_startswith(month, __DATE__);
        }

        template<char C>
        inline constexpr int to_int();

        #define MAKE_INT_CONVERTER(Val) template<> inline constexpr int to_int<#Val [0]>() { return Val; }

        template<>
        inline constexpr int to_int<' '>() { return 0; }
        MAKE_INT_CONVERTER(0);
        MAKE_INT_CONVERTER(1);
        MAKE_INT_CONVERTER(2);
        MAKE_INT_CONVERTER(3);
        MAKE_INT_CONVERTER(4);
        MAKE_INT_CONVERTER(5);
        MAKE_INT_CONVERTER(6);
        MAKE_INT_CONVERTER(7);
        MAKE_INT_CONVERTER(8);
        MAKE_INT_CONVERTER(9);

        inline constexpr int today()
        {
            return __DATE__[5] == ' ' ? to_int<__DATE__[4]>() :
                10 * to_int<__DATE__[4]>() + to_int<__DATE__[5]>();
        }

        #define REMOVE_BEFORE(MONTH, DAY, MESSAGE) static_assert(::mage::compile_time::today() < DAY && ::mage::compile_time::this_month(#MONTH), MESSAGE)
    }
}
