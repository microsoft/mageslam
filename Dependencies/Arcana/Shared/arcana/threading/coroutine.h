// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#ifdef __cpp_coroutines

#include <assert.h>

#include <arcana/threading/task.h>
#include <experimental/coroutine>

namespace mira
{
    namespace
    {
        expected<void> coroutine_success = expected<void>::make_valid();
    }

    namespace internal
    {
        class unobserved_error : public std::system_error
        {
        public:
            unobserved_error(std::error_code error) : std::system_error(error) {}
        };

        template <typename ResultT>
        inline void HandleCoroutineException(std::exception_ptr e, mira::task_completion_source<ResultT>& taskCompletionSource)
        {
            try
            {
                std::rethrow_exception(e);
            }
            catch (const unobserved_error& error)
            {
                taskCompletionSource.complete(error.code());
            }
            catch (...)
            {
                assert(false && "Unhandled exception. Arcana task returning functions should handle exceptions as arcana tasks do not support them.");
                std::terminate();
            }
        }
    }

    // NOTE: Ideally, we want something like the below to support task<void> returning coroutines
    // to just co_return without a return value. However, the promise_type can't have both a
    // return_void and a return_value, which means it would not be able to support doing a co_return
    // of an std::error_code. Because of this, we instead have to always co_return a value, and in the
    // case of a task<void> coroutine that is successful, we have to return coroutine_success.
    // Alternatively, we could use exceptions like most other coroutine adapters, in which case we
    // would not co_return std:error_code, but rather would throw an std::system_error, and coroutines
    // would need to try/catch exceptions, or let them bubble out to the returned task.

    // This enables generating a task<void> return value from a coroutine. For example:
    // task<void> DoSomethingAsync()
    // {
    //     co_return;
    // }
    //template <typename... Args>
    //struct std::experimental::coroutine_traits<task<void>, Args...>
    //{
    //    class promise_type
    //    {
    //    public:
    //        auto get_return_object() { return m_taskCompletionSource.as_task(); }
    //        std::experimental::suspend_never initial_suspend() { return {}; }
    //        std::experimental::suspend_never final_suspend() { return {}; }
    //        void set_exception(std::exception_ptr e) { mira::internal::HandleCoroutineException(e, m_taskCompletionSource); }
    //        void return_void() { m_taskCompletionSource.complete(); }
    //    private:
    //        mira::task_completion_source<void> m_taskCompletionSource;
    //    };
    //};

    // This enables generating a task<ResultT> return value from a coroutine. For example:
    // task<int> DoSomethingAsync()
    // {
    //     co_return 42;
    // }
    template <typename ResultT, typename... Args>
    struct std::experimental::coroutine_traits<task<ResultT>, Args...>
    {
        class promise_type
        {
        public:
            auto get_return_object() { return m_taskCompletionSource.as_task(); }
            std::experimental::suspend_never initial_suspend() { return {}; }
            std::experimental::suspend_never final_suspend() { return {}; }
            void set_exception(std::exception_ptr e) { mira::internal::HandleCoroutineException(e, m_taskCompletionSource); }

            template<typename T>
            void return_value(T&& result) { m_taskCompletionSource.complete(std::forward<T>(result)); }
        private:
            mira::task_completion_source<ResultT> m_taskCompletionSource;
        };
    };

    namespace internal
    {
        // The task_awaiter_result class (plus void specialization) wrap an expected<ResultT> and ensure that any errors are observed.
        // If the expected<ResultT> is in an error state and an attempt is made to access the value, an unobserved_error exception is thrown.
        //   This supports scenairos where a value is directly obtained:
        //   int result = co_await SomeTaskOfIntReturningFunction();
        // If the expected<ResultT> is in an error state and is destroyed when the error has not been observed, an unobserved_error exception is thrown.
        //   This supports scenarios where the result of the co_await is ignored:
        //   co_await SomeTaskReturningFunctionThatResultsInAnError(); 
        // This is propagated up to the coroutine_traits, and in the case of the mira task coroutine_traits, the final task is set to an error
        // state and contains the unobserved error (thereby propagating the error all the way to the caller).
        template <typename ResultT>
        class task_awaiter_result
        {
        public:
            task_awaiter_result(expected<ResultT>&& expected) :
                m_expected(std::move(expected))
            {
            }

            task_awaiter_result(task_awaiter_result&& other) :
                m_expected(std::move(other.m_expected)),
                m_observed(other.m_observed)
            {
                other.m_observed = true;
            }

            task_awaiter_result(const task_awaiter_result&) = delete;
            task_awaiter_result& operator=(const task_awaiter_result&) = delete;

            ~task_awaiter_result() noexcept(false)
            {
                if (!m_observed && m_expected.has_error())
                {
                    throw unobserved_error(m_expected.error());
                }
            }

            operator mira::expected<ResultT>()
            {
                m_observed = true;
                return m_expected;
            }

            operator ResultT()
            {
                m_observed = true;
                if (m_expected.has_error())
                {
                    throw unobserved_error(m_expected.error());
                }

                return m_expected.value();
            }

        private:
            expected<ResultT> m_expected;
            bool m_observed{ false };
        };

        template<>
        class task_awaiter_result<void>
        {
        public:
            task_awaiter_result(expected<void>&& expected) :
                m_expected(std::move(expected))
            {
            }

            task_awaiter_result(task_awaiter_result&& other) :
                m_expected(std::move(other.m_expected)),
                m_observed(other.m_observed)
            {
                other.m_observed = true;
            }

            task_awaiter_result(const task_awaiter_result&) = delete;
            task_awaiter_result& operator=(const task_awaiter_result&) = delete;

            ~task_awaiter_result() noexcept(false)
            {
                if (!m_observed && m_expected.has_error())
                {
                    throw unobserved_error(m_expected.error());
                }
            }

            operator mira::expected<void>()
            {
                m_observed = true;
                return m_expected;
            }

        private:
            expected<void> m_expected;
            bool m_observed{ false };
        };
    }

    // This enables awaiting a task<ResultT> within a coroutine. For example:
    // std::future<int> DoAnotherThingAsync()
    // {
    //     auto result = co_await configure_await(mira::inline_scheduler, DoSomethingAsync());
    //     return result.value();
    // }
    template <typename SchedulerT, typename ResultT>
    inline auto configure_await(SchedulerT& scheduler, task<ResultT> task)
    {
        class task_awaiter
        {
        public:
            task_awaiter(SchedulerT& scheduler, mira::task<ResultT> task) : m_scheduler(scheduler), m_task(std::move(task)) {}
            bool await_ready() { return false; }
            auto await_resume() { return mira::internal::task_awaiter_result<ResultT>(std::move(m_result)); }
            void await_suspend(std::experimental::coroutine_handle<> coroutine)
            {
                m_task.then(m_scheduler, mira::cancellation::none(), [this, coroutine = std::move(coroutine)](const mira::expected<ResultT>& result)
                {
                    m_result = result;
                    coroutine.resume();
                });
            }
        private:
            SchedulerT& m_scheduler;
            mira::task<ResultT> m_task;
            mira::expected<ResultT> m_result;
        };

        return task_awaiter{ scheduler, std::move(task) };
    }

    // This enables awaiting a scheduler (e.g. switching scheduler/dispatcher contexts). For example:
    // std::future<void> DoSomethingAsync()
    // {
    //    // do some stuff in the current scheduling context
    //    co_await switch_to(background_dispatcher);
    //    // do some stuff on a background thread
    //    co_await switch_to(render_dispatcher);
    //    // do some stuff on the render thread
    // }
    template <typename SchedulerT>
    inline auto switch_to(SchedulerT& scheduler)
    {
        class scheduler_awaiter
        {
        public:
            scheduler_awaiter(SchedulerT& scheduler) : m_scheduler(scheduler) {}
            bool await_ready() { return false; }
            void await_resume() {}
            void await_suspend(std::experimental::coroutine_handle<> coroutine)
            {
                m_scheduler.queue([coroutine = std::move(coroutine)]
                {
                    coroutine.resume();
                });
            }
        private:
            SchedulerT& m_scheduler;
        };

        return scheduler_awaiter{ scheduler };
    }
}
#endif
