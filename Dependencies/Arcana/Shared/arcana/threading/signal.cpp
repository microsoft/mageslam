// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "signal.h"
#include "cancellation.h"
#include <chrono>          // for milliseconds

namespace mira
{
    template<typename FutureT>
    void basic_signal<FutureT>::wait(const cancellation& token, bool autoreset)
    {
        std::future_status status = std::future_status::timeout;
        while (!token.cancelled() && status == std::future_status::timeout)
        {
            status = m_wait.wait_for(std::chrono::milliseconds{ 10 });
        }

        if (autoreset)
        {
            reset();
        }
    }

    // template instantiations
    template class basic_signal<std::future<void>>;
    template class basic_signal<std::shared_future<void>>;
}
