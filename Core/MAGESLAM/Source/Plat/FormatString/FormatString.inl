// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include <boost/format.hpp>

namespace Restricted
{
    inline boost::format& formatStringImpl(boost::format& format)
    {
        return format;
    }

    template <typename First, typename... Rest>
    inline boost::format& formatStringImpl(boost::format& format, First const& first, Rest&&... rest)
    {
        return formatStringImpl(format % first, std::forward<Rest>(rest)...);
    }

    inline boost::wformat& formatStringImpl(boost::wformat& format)
    {
        return format;
    }

    template <typename First, typename... Rest>
    inline boost::wformat& formatStringImpl(boost::wformat& format, First const& first, Rest&&... rest)
    {
        return formatStringImpl(format % first, std::forward<Rest>(rest)...);
    }
}
