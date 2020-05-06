// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <codecvt>
#include <string>
#include <locale>

#include <gsl/gsl>

namespace mira
{
    inline std::string utf16_to_utf8(const wchar_t* input)
    {
        std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
        return converter.to_bytes(input);
    }

    inline std::wstring utf8_to_utf16(const char* input)
    {
        std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
        return converter.from_bytes(input);
    }

    inline std::string utf16_to_utf8(const std::wstring& input)
    {
        return utf16_to_utf8(input.data());
    }

    inline std::wstring utf8_to_utf16(const std::string& input)
    {
        return utf8_to_utf16(input.data());
    }

    struct string_compare
    {
        using is_transparent = std::true_type;

        bool operator()(gsl::cstring_span<> a, gsl::cstring_span<> b) const
        {
            return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
        }

        bool operator()(gsl::czstring_span<> a, gsl::czstring_span<> b) const
        {
            return (*this)(a.as_string_span(), b.as_string_span());
        }

        bool operator()(const char* a, const char* b) const
        {
            return strcmp(a, b) < 0;
        }
    };
}
