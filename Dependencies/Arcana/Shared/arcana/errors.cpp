// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "errors.h"

#include <assert.h>
#include <sstream>

namespace mira
{
    const char* generic_error_category::name() const noexcept
    {
        return "mira::generic_error_category";
    }

    std::string generic_error_category::message(int evt) const
    {
        switch (static_cast<errc>(evt))
        {
        case errc::unexpected:
            return "unexpected";
        case errc::not_enough_input:
            return "not enough input";
        case errc::failed:
            return "failed";
        case errc::skipped:
            return "skipped";
        default:
            assert(false);
            return "unknown error";
        }
    }

    const std::error_category& generic_category()
    {
        static generic_error_category cat;
        return cat;
    }

    // Converts the given std::system_error to a readable string.
    // For instance => throw std::system_error(std::make_error_code(std::errc::invalid_argument), "Could not find a
    // matching source."); woud become  => Exception details. Code:[generic:22] What:[Could not find a matching source.
    // Invalid argument]
    std::string to_string(const std::system_error& error)
    {
        std::stringstream exceptionMessage;

        exceptionMessage << "Exception Details. Code:[" << error.code() << "] What:[" << error.what() << "]";

        return exceptionMessage.str();
    }
}
