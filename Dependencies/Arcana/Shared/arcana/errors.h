// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <system_error>

namespace mira
{
    enum class errc
    {
        unexpected = 1,
        not_enough_input,
        failed,
        skipped
    };

    class generic_error_category : public std::error_category
    {
    public:
        virtual const char* name() const noexcept override;
        virtual std::string message(int evt) const override;
    };

    const std::error_category& generic_category();

    std::string to_string(const std::system_error& error);
}

namespace std
{
    template<>
    struct is_error_code_enum<mira::errc> : public true_type
    {
    };

    inline std::error_code make_error_code(::mira::errc e)
    {
        return std::error_code(static_cast<int>(e), ::mira::generic_category());
    }
}
