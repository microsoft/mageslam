// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    class not_implemented_exception : public std::logic_error
    {
    public:
        not_implemented_exception()
            : std::logic_error{ nullptr }
        {}

        not_implemented_exception(const char* const message)
            : std::logic_error(message)
        {}
    };
}
