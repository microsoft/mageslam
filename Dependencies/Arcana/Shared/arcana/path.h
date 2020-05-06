// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#if __has_include(<experimental/filesystem>)

#include <experimental/filesystem>

namespace mira
{
    using path = std::experimental::filesystem::path;
}

#else

namespace mira
{
    class path
    {
    public:
        path(const std::string& value)
            : m_value{ value }
        {}

        const std::string& string() const
        {
            return m_value;
        }
    private:
        std::string m_value;
    };

    inline std::ostream& operator <<(std::ostream& stream, const path& out)
    {
        stream << out.string();
        return stream;
    }

    inline std::istream& operator >>(std::istream& stream, path& out)
    {
        std::string value;
        stream >> value;

        out = path{ value };

        return stream;
    }
}

#endif
