// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/messaging/router.h>
#include <arcana/threading/cancellation.h>
#include <arcana/threading/pending_task_scope.h>

#include "Utils/thread_memory.h"

namespace mage
{
    class BaseWorker
    {
    public:
        mira::task<void> DisposeAsync();

    protected:
        BaseWorker(size_t heapSize, size_t stackSize)
            : m_memory{ heapSize, stackSize }
        {}

        ~BaseWorker() = default;

        mira::cancellation& Cancellation()
        {
            return m_cancellation;
        }

        memory_pool& MemoryPool()
        {
            return m_memory;
        }

        mira::ticket_scope& Registrations()
        {
            return m_registrations;
        }

        mira::pending_task_scope& Pending()
        {
            return m_pendingWork;
        }

    private:
        // template method for deriving class to run custom dispose logic
        virtual mira::task<void> OnDisposeAsync()
        {
            return mira::task_from_result();
        }

        memory_pool m_memory;

        mira::cancellation_source m_cancellation{};
        mira::pending_task_scope m_pendingWork{};
        mira::ticket_scope m_registrations{};
    };
}
