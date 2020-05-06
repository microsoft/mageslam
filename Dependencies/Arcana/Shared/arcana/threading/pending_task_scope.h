// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "task.h"

#include <vector>

namespace mira
{
    class pending_task_scope
    {
    public:
        template<typename ResultT>
        task<ResultT>& operator+=(task<ResultT>&& task)
        {
            return *this += task;
        }

        template<typename ResultT>
        task<ResultT>& operator+=(task<ResultT>& task)
        {
            {
                std::lock_guard<std::mutex> guard{ m_mutex };
                if (m_disposing)
                    throw std::runtime_error("can't add tasks to a pending scope once it starts disposing itself");

                m_pending++;
            }

            task.then(inline_scheduler, cancellation::none(), [this](const expected<ResultT>& result)
            {
                bool complete = false;
                {
                    std::lock_guard<std::mutex> guard{ m_mutex };
                    m_pending--;

                    complete = m_disposing && m_pending == 0;

                    // If this failed and is the first error, copy that error
                    // as further errors are likely due to cascading failures.
                    if (result.has_error() && !m_error)
                    {
                        m_error = result.error();
                    }

                    if (complete)
                    {
                        m_setcompletion = true;
                    }
                }

                if (complete)
                {
                    complete_internal();
                }
            });

            return task;
        }

        bool completed() const noexcept
        {
            return m_pending == 0;
        }

        bool has_error() const noexcept
        {
            return static_cast<bool>(m_error);
        }

        const std::error_code& error() const noexcept
        {
            return m_error;
        }

        task<void> when_all()
        {
            {
                std::lock_guard<std::mutex> guard{ m_mutex };

                m_disposing = true;
                if (m_pending == 0 && !m_setcompletion)
                {
                    m_setcompletion = true;
                    complete_internal();
                }
            }
            return m_completed.as_task();
        }

        ~pending_task_scope()
        {
            assert(completed() && "a task scope has to outlive all its tasks, you can't destroy it's owner until all "
                                  "its tasks are done.");
        }

    private:
        void complete_internal()
        {
            if (m_error)
            {
                m_completed.complete(m_error);
            }
            else
            {
                m_completed.complete();
            }
        }

        mutable std::mutex m_mutex;
        int m_pending = 0;
        bool m_disposing = false;
        bool m_setcompletion = false;
        task_completion_source<void> m_completed;
        std::error_code m_error;
    };
}
