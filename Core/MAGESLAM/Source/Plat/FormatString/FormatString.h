// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "FormatString.inl"

namespace mira
{
    template <typename... Args>
    inline std::string FormatString(const std::string& formatString, Args&&... args)
    {
        boost::format format(formatString);
        return Restricted::formatStringImpl(format, std::forward<Args>(args)...).str();
    }

    template <typename... Args>
    inline std::wstring FormatString(const std::wstring& formatString, Args&&... args)
    {
        boost::wformat format(formatString);
        return Restricted::formatStringImpl(format, std::forward<Args>(args)...).str();
    }
}
