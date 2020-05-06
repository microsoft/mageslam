// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "BaseWorker.h"

namespace mage
{
    mira::task<void> BaseWorker::DisposeAsync()
    {
        m_registrations.clear();
        m_cancellation.cancel();

        return m_pendingWork.when_all().then(mira::inline_scheduler, mira::cancellation::none(),
            [this](const mira::expected<void>&)
            {
                return OnDisposeAsync();
            });
    }
}
