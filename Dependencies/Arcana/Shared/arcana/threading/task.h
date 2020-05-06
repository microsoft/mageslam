// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/errors.h"
#include "arcana/expected.h"
#include "arcana/functional/inplace_function.h"
#include "arcana/iterators.h"
#include "arcana/type_traits.h"

#include "cancellation.h"

#include <gsl/gsl>
#include <memory>
#include <stdexcept>

#include <atomic>

namespace mira
{
    template<typename ResultT>
    class task_completion_source;

    //
    // A scheduler that will invoke the continuation inline
    // right after the previous task.
    //
    namespace
    {
        struct inline_scheduler_t
        {
            template<typename CallableT>
            constexpr void queue(CallableT&& callable) const
            {
                callable();
            }
        };

        constexpr inline_scheduler_t inline_scheduler{};
    }
}

#include "internal/internal_task.h"

namespace mira
{
    //
    //  Generic task system to run work with continuations on a generic scheduler.
    //
    //  The scheduler on which tasks are queued must satisfy this contract:
    //
    //      struct scheduler
    //      {
    //          template<CallableT>
    //          void queue(CallableT&& callable)
    //          {
    //              callable must be usable like this: callable();
    //          }
    //      };
    //
    template<typename ResultT>
    class task
    {
        using payload_t = internal::base_task_payload_with_return<ResultT>;
        using payload_ptr = std::shared_ptr<payload_t>;

        static_assert(std::is_same<typename as_expected<ResultT>::result, ResultT>::value,
            "task can't be of expected<T>");

    public:
        using result_t = ResultT;

        task() = default;

        task(const task& other) = default;
        task(task&& other) = default;

        task(const task_completion_source<ResultT>& source)
            : m_payload{ source.m_payload }
        {}

        task(task_completion_source<ResultT>&& source)
            : m_payload{ std::move(source.m_payload) }
        {}

        task& operator=(task&& other) = default;
        task& operator=(const task& other) = default;

        bool operator==(const task& other)
        {
            return m_payload == other.m_payload;
        }

        //
        // Executes a callable on this scheduler once this task is finished and
        // returns a task that represents the callable.
        //
        // Calling .then() on the returned task will queue a task to run after the
        // callable is run.
        //
        template<typename SchedulerT, typename CallableT>
        auto then(SchedulerT& scheduler, cancellation& token, CallableT&& callable)
        {
            return internal::continuation_factory<ResultT>::create_continuation_task(
                internal::input_output_wrapper<ResultT>::wrap_callable(std::forward<CallableT>(callable), token),
                scheduler,
                m_payload);
        }

    private:
        explicit task(payload_ptr payload)
            : m_payload{ std::move(payload) }
        {}

        template<typename CallableT, typename InputT>
        explicit task(CallableT&& callable, type_of<InputT> input)
        {
            auto payload = std::make_shared<internal::task_payload<ResultT, sizeof(CallableT)>>(
                std::forward<CallableT>(callable), input);

            m_payload = std::move(payload);
        }

        template<typename OtherResultT, size_t WorkSize>
        friend struct internal::task_payload;

        template<typename OtherResultT>
        friend class task;

        friend class task_completion_source<ResultT>;

        template<typename OtherResultT, typename InputT>
        friend struct internal::task_factory;

        template<typename OtherResultT>
        friend struct internal::continuation_factory;

        template<typename SchedulerT, typename CallableT>
        friend auto make_task(SchedulerT& scheduler, cancellation& token, CallableT&& callable) -> typename internal::task_factory<typename as_expected<decltype(callable())>::result, void>::task_t;

        template<typename OtherResultT>
        friend task<typename as_expected<OtherResultT>::result> task_from_result(OtherResultT&& value);

        friend task<void> task_from_result();

        template<typename OtherResultT, typename ErrorT>
        friend task<OtherResultT> task_from_error(const ErrorT& error);

        payload_ptr m_payload;
    };
}

namespace mira
{
    template<typename ResultT>
    class task_completion_source
    {
        using payload_t = internal::noop_task_payload<ResultT>;
        using payload_ptr = std::shared_ptr<payload_t>;
        using uninitialized = std::integral_constant<int, 0>;

    public:
        using result_t = ResultT;

        task_completion_source()
            : m_payload{ std::make_shared<payload_t>() }
        {}

        static task_completion_source<ResultT> make_uninitialized()
        {
            return task_completion_source<ResultT>{ uninitialized{} };
        }

        //
        // Completes the task this source represents.
        //
        void complete()
        {
            static_assert(std::is_same<ResultT, void>::value,
                "complete with no arguments can only be used with a void completion source");
            m_payload->complete(expected<void>::make_valid());
        }

        //
        // Completes the task this source represents.
        //
        template<typename ValueT>
        void complete(ValueT&& value)
        {
            m_payload->complete(std::forward<ValueT>(value));
        }

        //
        // Returns whether or not the current source has already been completed.
        //
        bool completed() const
        {
            return m_payload->completed();
        }

        //
        // Converts this task_completion_source to a task object for consumers to use.
        //
        task<ResultT> as_task() const &
        {
            return task<ResultT>{ m_payload };
        }

        task<ResultT> as_task() &&
        {
            return task<ResultT>{ std::move(m_payload) };
        }

    private:
        explicit task_completion_source(uninitialized)
        {}

        explicit task_completion_source(std::shared_ptr<payload_t> payload)
            : m_payload{ std::move(payload) }
        {}

        friend class task<ResultT>;
        friend class abstract_task_completion_source;

        template<typename R, typename I>
        friend struct internal::task_factory;

        payload_ptr m_payload;
    };

    //
    // a type erased version of task_completion_source.
    //
    class abstract_task_completion_source
    {
        using payload_t = internal::base_task_payload;
        using payload_ptr = std::shared_ptr<payload_t>;

    public:
        abstract_task_completion_source()
            : m_payload{}
        {}

        template<typename T>
        explicit abstract_task_completion_source(const task_completion_source<T>& other)
            : m_payload{ other.m_payload }
        {}

        template<typename T>
        explicit abstract_task_completion_source(task_completion_source<T>&& other)
            : m_payload{ std::move(other.m_payload) }
        {}

        //
        // Returns whether or not the current source has already been completed.
        //
        bool completed() const
        {
            return m_payload->completed();
        }

        template<typename T>
        bool operator==(const task_completion_source<T>& other)
        {
            return m_payload == other.m_payload;
        }

        template<typename T>
        task_completion_source<T> unsafe_cast()
        {
            return task_completion_source<T>{ std::static_pointer_cast<internal::noop_task_payload<T>>(m_payload) };
        }

    private:
        payload_ptr m_payload;
    };

    //
    // creates a task and queues it to run on the given scheduler
    //
    template<typename SchedulerT, typename CallableT>
    inline auto make_task(SchedulerT& scheduler, cancellation& token, CallableT&& callable) -> typename internal::task_factory<typename as_expected<decltype(callable())>::result, void>::task_t
    {
        using callable_return_t = typename as_expected<decltype(callable())>::result;
        using wrapper = internal::input_output_wrapper<void>;

        internal::task_factory<callable_return_t, void> factory(
            wrapper::wrap_callable(std::forward<CallableT>(callable), token));

        scheduler.queue([to_run = std::move(factory.to_run)]{ to_run.m_payload->run(nullptr); });

        return factory.to_return;
    }

    //
    // creates a completed task from the given result
    //
    template<typename ResultT>
    inline task<typename as_expected<ResultT>::result> task_from_result(ResultT&& value)
    {
        task_completion_source<typename as_expected<ResultT>::result> result;
        result.complete(std::forward<ResultT>(value));
        return std::move(result);
    }

    inline task<void> task_from_result()
    {
        task_completion_source<void> result;
        result.complete();
        return std::move(result);
    }

    template<typename ResultT, typename ErrorT>
    inline task<ResultT> task_from_error(const ErrorT& error)
    {
        task_completion_source<ResultT> result;
        result.complete(std::make_error_code(error));
        return std::move(result);
    }

    template<typename ResultT>
    inline task<ResultT> task_from_error(const std::error_code& error)
    {
        task_completion_source<ResultT> result;
        result.complete(error);
        return std::move(result);
    }

    inline task<void> when_all(gsl::span<task<void>> tasks)
    {
        if (tasks.empty())
        {
            return task_from_result();
        }

        struct when_all_data
        {
            std::mutex mutex;
            size_t pendingCount;
            std::error_code error;
        };

        task_completion_source<void> result;
        auto data = std::make_shared<when_all_data>();
        data->pendingCount = tasks.size();

        for (task<void>& task : tasks)
        {
            task.then(inline_scheduler, cancellation::none(), [data, result](const expected<void>& exp) mutable {
                bool last = false;
                {
                    std::lock_guard<std::mutex> guard{ data->mutex };

                    data->pendingCount -= 1;
                    last = data->pendingCount == 0;

                    if (exp.has_error() && !data->error)
                    {
                        // set the first error, as it might have cascaded
                        data->error = exp.error();
                    }
                }

                if (last) // we were the last task to complete
                {
                    if (data->error)
                    {
                        result.complete(data->error);
                    }
                    else
                    {
                        result.complete();
                    }
                }
            });
        }

        return std::move(result);
    }

    template<typename T>
    inline task<std::vector<T>> when_all(gsl::span<task<T>> tasks)
    {
        if (tasks.empty())
        {
            return task_from_result<std::vector<T>>(std::vector<T>());
        }

        struct when_all_data
        {
            std::mutex mutex;
            size_t pendingCount;
            std::error_code error;
            std::vector<T> results;
        };

        task_completion_source<std::vector<T>> result;
        auto data = std::make_shared<when_all_data>();
        data->pendingCount = tasks.size();
        data->results.resize(tasks.size());

        //using forloop with index to be able to keep proper order of results
        for (auto idx = 0U; idx < data->results.size(); idx++)
        {
            tasks[idx].then(mira::inline_scheduler, cancellation::none(), [data, result, idx](const expected<T>& exp) mutable {
                bool last = false;
                {
                    std::lock_guard<std::mutex> guard{ data->mutex };

                    data->pendingCount -= 1;
                    last = data->pendingCount == 0;

                    if (exp.has_error() && !data->error)
                    {
                        // set the first error, as it might have cascaded
                        data->error = exp.error();
                    }
                    if (exp.has_value())
                    {
                        data->results[idx] = exp.value();
                    }
                }

                if (last) // we were the last task to complete
                {
                    if (data->error)
                    {
                        result.complete(data->error);
                    }
                    else
                    {
                        result.complete(data->results);
                    }
                }
            });
        }

        return std::move(result);
    }

    template<typename... ArgTs>
    inline task<std::tuple<typename mira::void_passthrough<ArgTs>::type...>> when_all(task<ArgTs>... tasks)
    {
        using void_passthrough_tuple = std::tuple<typename mira::void_passthrough<ArgTs>::type...>;

        struct when_all_data
        {
            std::mutex mutex;
            int pending;
            std::error_code error;
            void_passthrough_tuple results;
        };
  
        task_completion_source<void_passthrough_tuple> result;

        auto data = std::make_shared<when_all_data>();
        data->pending = std::tuple_size<void_passthrough_tuple>::value;

        std::tuple<task<ArgTs>&...> taskrefs = std::make_tuple(std::ref(tasks)...);

        iterate_tuple(taskrefs, [&](auto& task, auto idx) {
            using task_t = std::remove_reference_t<decltype(task)>;

            task.then(inline_scheduler,
                cancellation::none(),
                [data, result](const expected<typename task_t::result_t>& exp) mutable {
                bool last = false;
                {
                    std::lock_guard<std::mutex> guard{ data->mutex };

                    data->pending -= 1;
                    last = data->pending == 0;

                    internal::write_expected_to_tuple<decltype(idx)::value>(data->results, exp);
                    
                    if (exp.has_error() && !data->error)
                    {
                        // set the first error, as it might have cascaded
                        data->error = exp.error();
                    }
                }

                if (last) // we were the last task to complete
                {
                    if (data->error)
                    {
                        result.complete(data->error);
                    }
                    else
                    {
                        result.complete(std::move(data->results));
                    }
                }
            });
        });
        return std::move(result);
    }
}
