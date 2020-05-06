// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/functional/inplace_function.h"
#include "arcana/type_traits.h"

#include <mutex>
#include <thread>

namespace mira
{
    template<typename ResultT>
    class task;

    namespace internal
    {
        struct base_task_payload
        {
        public:
            using work_function_t = void(*)(base_task_payload& payload, base_task_payload* parent);

            //
            // A task has 0-n continuations that run once it's done and that take
            // the result of the task as input parameters.
            //
            // A continuation doesn't necessarily run on the same scheduler as the task
            // it depends on. Since we need to keep these continuations in a homogeneous container
            // we type erase the scheduler by creating a callable that queues the run
            // method of the continuation on the right scheduler.
            //
            // We keep a weak_ptr to the parent because before the parent has run
            // it's holding on to the payload which would create a circular shared_ptr reference.
            // Once we queue the continuation we lock the weak_ptr in order to keep the
            // parent alive long enough for us to run and read its result. This doesn't create a circular
            // reference because it passes the ownership to the lambda that runs the continuation.
            //
            // The continuation task is the task payload that represents the task we need to run
            // when the previous one has been run.
            // 
            struct continuation_payload
            {
                explicit operator bool() const noexcept
                {
                    return continuation != nullptr;
                }

                continuation_payload() = default;

                template<typename SchedulerT>
                continuation_payload(SchedulerT& scheduler, std::weak_ptr<base_task_payload> parentArg, std::shared_ptr<base_task_payload> continuationArg)
                    : parent{ std::move(parentArg) }, continuation{ std::move(continuationArg) }
                {
                    assert(parent.lock() && "parent of a continuation can't be null");

                    queue = [&scheduler](queue_function&& func)
                    {
                        scheduler.queue(std::move(func));
                    };
                }

                void reparent(const std::shared_ptr<base_task_payload>& newParent)
                {
                    assert(newParent && "tried reparenting with a null parent");
                    parent = newParent;
                }

                void run()
                {
                    assert(parent.lock() && "parent of a continuation can't be null");

                    queue([shared = parent.lock(), continuation = std::move(continuation)]
                    {
                        assert(shared.get() && "parent of a continuation can't be null");

                        continuation->run(shared.get());
                    });
                }

            private:
                std::weak_ptr<base_task_payload> parent;
                std::shared_ptr<base_task_payload> continuation;

                using queue_function = stdext::inplace_function<void(), 2 * sizeof(std::shared_ptr<void>)>;
                stdext::inplace_function<void(queue_function&& func)> queue;
            };

            base_task_payload(work_function_t work)
                : m_work{ work }
            {}

            bool completed() const
            {
                return m_completed;
            }

            bool is_task_completion_source() const
            {
                return m_work == nullptr;
            }

            void run(base_task_payload* parent)
            {
                if (m_work != nullptr)
                {
                    m_work(*this, parent);
                }

                do_completion();
            }

            template<typename SchedulerT>
            void create_continuation(
                SchedulerT& scheduler,
                std::weak_ptr<base_task_payload> self,
                std::shared_ptr<base_task_payload> continuation)
            {
                add_continuation(continuation_payload{ scheduler, std::move(self), std::move(continuation) });
            }

            static void collapse_left_into_right(base_task_payload& left, const std::shared_ptr<base_task_payload>& right)
            {
                auto continuations = left.cannibalize(right);
                right->add_continuations(continuation_span(continuations), right);
            }

            void complete()
            {
                do_completion();
            }

        private:
            either<continuation_payload, std::vector<continuation_payload>> cannibalize(std::shared_ptr<base_task_payload> taskRedirect)
            {
                std::lock_guard<std::mutex> guard{ m_mutex };

                if (m_completed)
                    throw std::runtime_error("tried to complete a task twice");

                auto complete = gsl::finally([this] { m_completed = true; });

                m_taskRedirect = std::move(taskRedirect);

                // we need to clear the continuation once it's done because it
                // holds a reference to the payload during the transition period
                // and if we don't we'd create a circular shared_ptr reference and never destroy
                // the payload instances.
                return std::move(m_continuation);
            }

            static gsl::span<continuation_payload> continuation_span(either<continuation_payload, std::vector<continuation_payload>>& either)
            {
                if (either.has_second())
                {
                    return gsl::make_span(either.second());
                }
                else if (either.first())
                {
                    return gsl::make_span(&either.first(), 1);
                }
                else
                {
                    return {};
                }
            }

            void add_continuation(continuation_payload&& continuation)
            {
                add_continuations(gsl::make_span<continuation_payload>(&continuation, 1));
            }

            void add_continuations(gsl::span<continuation_payload> continuations, const std::shared_ptr<base_task_payload>& self)
            {
                assert(self.get() == this && "when adding continuations through cannibalization we need to reparent the continuations to ourselves");

                if (continuations.empty())
                    return;

                for (auto& continuation : continuations)
                    continuation.reparent(self);

                add_continuations(continuations);
            }

            void add_continuations(gsl::span<continuation_payload> continuations)
            {
                if (continuations.empty())
                    return;

                bool runit = false;

                {
                    std::lock_guard<std::mutex> guard{ m_mutex };

                    if (m_taskRedirect)
                    {
                        for (auto& continuation : continuations)
                            continuation.reparent(m_taskRedirect);

                        m_taskRedirect->add_continuations(continuations);
                    }
                    else
                    {
                        if (m_completed)
                        {
                            runit = true;
                        }
                        else
                        {
                            internal_add_continuations(continuations);
                        }
                    }
                }

                if (runit)
                {
                    for (auto& continuation : continuations)
                        continuation.run();
                }
            }

            void do_completion()
            {
                either<continuation_payload, std::vector<continuation_payload>> continuation = cannibalize(nullptr);

                for (auto& callable : continuation_span(continuation))
                    callable.run();
            }

            void internal_add_continuations(continuation_payload&& continuation)
            {
                internal_add_continuations(gsl::make_span(&continuation, 1));
            }

            void internal_add_continuations(gsl::span<continuation_payload> continuations)
            {
                if (m_completed)
                    throw std::runtime_error("already specified a continuation for the current task");

                assert(!continuations.empty() && "we shouldn't be calling this with an empty continuation set");

                if (m_continuation.has_second())
                {
                    m_continuation.second().insert(
                        m_continuation.second().end(), std::make_move_iterator(continuations.begin()), std::make_move_iterator(continuations.end()));
                }
                else if (!m_continuation.first() && continuations.size() == 1)
                {
                    m_continuation.first() = std::move(continuations.at(0));
                }
                else
                {
                    // create a vector with the existing continuation (if there is one)
                    // then append the new ones.
                    if (m_continuation.first())
                    {
                        m_continuation = std::vector<continuation_payload>{ std::move(m_continuation.first()) };

                        m_continuation.second().insert(m_continuation.second().end(), std::make_move_iterator(continuations.begin()), std::make_move_iterator(continuations.end()));
                    }
                    else
                    {
                        m_continuation = std::vector<continuation_payload>(
                            std::make_move_iterator(continuations.begin()), std::make_move_iterator(continuations.end()));
                    }
                }
            }

        protected:
            // TODO make sure we don't have huge gaps in the object layout
            std::mutex m_mutex;

        private:
            bool m_completed = false;
            work_function_t m_work = nullptr;
            either<continuation_payload, std::vector<continuation_payload>> m_continuation{ continuation_payload{} };

            // when unwrapping multiple tasks of tasks we don't want to
            // create unbounded continuation chains. Which means we cannibalize
            // the top level task_completion_source and add all its continuations
            // to the task that gets returned from the async method. Once we cannibalize
            // the original task still exists, so if someone tries to add itself as a
            // continuation it won't ever get called or won't have the right result.
            // In order to get around that we set task redirect (like a forwarding address or an http 302 redirect)
            // which is the task returned by the async method that now represents the actual task that
            // will eventually get completed and contain the result that was meant for the
            // original task_completion_source. The m_taskRedirect task is then where we add the continuation instead
            // of the task_completion_source.
            std::shared_ptr<base_task_payload> m_taskRedirect;
        };

        template<typename ResultT>
        struct output_wrapper
        {
            template<typename CallableT, typename InputT>
            static typename as_expected<ResultT>::type invoke(CallableT&& callable, InputT&& input)
            {
                return callable(input);
            }

            template<typename CallableT>
            static typename as_expected<ResultT>::type invoke(CallableT&& callable)
            {
                return callable();
            }
        };

        template<>
        struct output_wrapper<void>
        {
            template<typename CallableT, typename InputT>
            static expected<void> invoke(CallableT&& callable, InputT&& input)
            {
                callable(input);

                return {};
            }

            template<typename CallableT>
            static expected<void> invoke(CallableT&& callable)
            {
                callable();

                return {};
            }
        };

        template<typename ResultT>
        struct base_task_payload_with_return : base_task_payload
        {
            expected<ResultT> Result;

            base_task_payload_with_return(work_function_t work)
                : base_task_payload{ work }
            {}

            void complete(expected<ResultT>&& result)
            {
                Result = std::move(result);

                base_task_payload::complete();
            }

            void complete(const expected<ResultT>& result)
            {
                Result = result;

                base_task_payload::complete();
            }
        };

        template<typename ResultT>
        struct noop_task_payload : base_task_payload_with_return<ResultT>
        {
            noop_task_payload()
                : base_task_payload_with_return<ResultT>{ nullptr }
            {}
        };

        //
        // helper struct to invoke a lambda if it has a parent task or not.
        //
        // Tasks require a parent if they received a type other than void as input.
        // So we specialize on the input type and if the work requires input, we assert
        // that the parent isn't null whereas if the input type is void, we use a valid
        // expected<void> to run the work. We should technically add some sort of validation
        // that a task with a null parent was a root task created by make_task as that should
        // be the only case where parent can be null.
        //
        template<typename ReturnT, typename InputT>
        struct task_invoker
        {
            template<typename WorkT>
            static expected<ReturnT> invoke(WorkT&& work, base_task_payload* baseParent)
            {
                assert(baseParent && "work that requires output from the parent task can't have a null parent");

                base_task_payload_with_return<InputT>* parent = static_cast<base_task_payload_with_return<InputT>*>(baseParent);
                return work(parent->Result);
            }
        };

        template<typename ReturnT>
        struct task_invoker<ReturnT, void>
        {
            template<typename WorkT>
            static expected<ReturnT> invoke(WorkT&& work, base_task_payload* baseParent)
            {
                if (baseParent)
                {
                    base_task_payload_with_return<void>* parent = static_cast<base_task_payload_with_return<void>*>(baseParent);
                    return work(parent->Result);
                }
                else
                {
                    return work(expected<void>::make_valid());
                }
            }
        };

        template<typename ResultT, size_t WorkSize>
        struct task_payload : base_task_payload_with_return<ResultT>
        {
            stdext::inplace_function<expected<ResultT>(base_task_payload*), WorkSize> Work;

            template<typename WorkFunction, typename InputT>
            task_payload(WorkFunction&& work, type_of<InputT>)
                : base_task_payload_with_return<ResultT>{ &do_work }
            {
                Work = [work = std::forward<WorkFunction>(work)](base_task_payload* baseParent) mutable
                {
                    return task_invoker<ResultT, InputT>::invoke(work, baseParent);
                };
            }

            static void do_work(base_task_payload& base, base_task_payload* baseParent)
            {
                auto& self = static_cast<task_payload&>(base);

                self.Result = self.Work(baseParent);
                self.Work = {}; // clear the work to destroy the callable
            }
        };

        //
        // task_factory is responsible for creating the task objects on continuations.
        // The task_factory<task> specialization is used to unwrap task<task> for callers.
        //
        template<typename ReturnT, typename InputT>
        struct task_factory
        {
            using task_t = task<ReturnT>;

            template<typename CallableT>
            task_factory(CallableT&& callable)
                : to_run{ std::forward<CallableT>(callable), type_of<InputT>{} }
                , to_return{ to_run }
            {}

            task_t to_run;
            task_t to_return;
        };

        template<typename ReturnT, typename InputT>
        struct task_factory<task<ReturnT>, InputT>
        {
            using task_t = task<ReturnT>;

            template<typename CallableT>
            task_factory(CallableT&& callable)
                : to_run{ std::forward<CallableT>(callable), type_of<InputT>{} }
            {
                task_completion_source<ReturnT> source;

                to_run.then(inline_scheduler, cancellation::none(), [source](const expected<task_t>& result) mutable {
                    if (result.has_error())
                    {
                        source.complete(result.error());
                    }
                    else
                    {
                        // Here when the result is also a task_completion_source
                        // we can collapse them to implement a sort of task tail recursion
                        // to remove unbounded task_completion_source chains.
                        // We achieve this by pulling out all the continuations from our
                        // task_completion_source which was a stand in for the task,
                        // and putting them on the actual task it was representing.
                        //
                        // We then set the forwarding task on the source in case
                        // someone is keeping it around and wants to put a continuation on
                        // it after the fact (in the cannibalize method). This ensures that
                        // anyone calling .then() on the stand-in task_completion_source
                        // we be added as a continuation to the right task.
                        //
                        // Then when we add the continuations to the task, we update their parent
                        // tasks (or where they get their results from) to the new task that was
                        // created in the callable. This ensures that the continuations use
                        // the result from the inner most task on completion which stores the actual result
                        // of the whole chain.
                        //
                        // The downside of the forwarding task is that if you keep a top level
                        // stand-in completion source around and call .then() on it after unwrapping
                        // you'll have to walk a potentially unbounded chain of forwarding tasks.
                        // Thankfully in all our scenarios that use infinite recursive tasks we immediately
                        // add a continuation to it and then await cancellation when we're done.
                        //
                        // One can think of this system like a platformer where the character
                        // is running on a platform that is advancing using pieces from its tail.
                        //
                        //   step 1:     step 2:   step 3:  (repeat)
                        //      __o        __o        __o 
                        //    _ \<_      _ \<_      _ \<_ 
                        //   (_)/(_)    (_)/(_)    (_)/(_)
                        //   a b c d     b c d     b c d a
                        //                 a
                        //

                        base_task_payload::collapse_left_into_right(*source.m_payload, result.value().m_payload);
                    }
                });

                to_return = std::move(source);
            }

            task<task_t> to_run;
            task_t to_return;
        };

        //
        // the continuation_factory is responsible for linking tasks together in continuations with their
        // respective result storage areas and invocations.
        //
        template<typename ResultT>
        struct continuation_factory
        {
            template<typename CallableT, typename SchedulerT>
            static auto create_continuation_task(CallableT&& callable,
                SchedulerT& scheduler,
                const std::shared_ptr<base_task_payload_with_return<ResultT>>& self)
            {
                using callable_return_t = decltype(callable(instance_of<expected<ResultT>>()));
                using callable_expected_return_t = typename as_expected<callable_return_t>::result;

                internal::task_factory<callable_expected_return_t, ResultT> factory(std::forward<CallableT>(callable));

                self->create_continuation(scheduler, self, std::move(factory.to_run.m_payload));

                return factory.to_return;
            }
        };

        //
        // Helper functions to determine whether or not a lambda handles mira::expected<T>.
        // It currently relies on the fact that expecteds can be built implicitly from an error_code
        // but this isn't exact. If someone were to pass in a lambda that takes an std::error_code,
        // things could get weird quick (the error message might be real bad).
        //
        template<typename CallableT>
        std::true_type handles_expected(CallableT&& callable,
                                        decltype(instance_of<CallableT>()(instance_of<std::error_code>())) * = nullptr);
        std::false_type handles_expected(...);

        template<typename ReturnT, typename CallableT, typename InputT>
        auto invoke_callable_that_handles_expected(CallableT& callable, cancellation& cancel, const expected<InputT>& input)
        {
            // Because the callable supports an expected<> input parameter
            // we need to call it if the previous task fails. But if the task doesn't
            // care about cancellation, and it's cancellation token is set then we
            // can just return the cancellation result directly.
            if ((!input.has_error() || input.error() == std::errc::operation_canceled) && cancel.cancelled())
                return typename as_expected<ReturnT>::type{ std::make_error_code(std::errc::operation_canceled) };

            return output_wrapper<ReturnT>::invoke(callable, input);
        }

        //
        // The input_output_wrapper types job is to grab an arbitrary lamba and adapt it
        // so that it returns an expected<ReturnT> and takes an expected<InputT>.
        //
        // While doing so it also adds the helper logic which takes care of cancellation
        // and error states for methods that use cancellation tokens and receive InputT
        // as a parameter directly.
        //
        template<typename InputT>
        struct input_output_wrapper
        {
            template<typename CallableT>
            static auto wrap_callable(CallableT&& callable, cancellation& cancel)
            {
                return wrap_input(std::forward<CallableT>(callable), cancel,
                                  decltype(handles_expected(instance_of<CallableT>())){});
            }

            template<typename CallableT>
            static auto wrap_input(CallableT&& callable, cancellation& cancel, std::true_type /*handles expected*/)
            {
                using callable_return_t = decltype(callable(mira::expected<InputT>{}));

                return[callable = std::forward<CallableT>(callable), &cancel](const expected<InputT>& input) mutable
                {
                    return invoke_callable_that_handles_expected<callable_return_t>(callable, cancel, input);
                };
            }

            template<typename CallableT>
            static auto wrap_input(CallableT&& callable, cancellation& cancel, std::false_type /*handles expected*/)
            {
                using callable_return_t = decltype(callable(std::declval<InputT>()));

                return [callable = std::forward<CallableT>(callable), &cancel](const expected<InputT>& input) mutable
                {
                    // Here the callable doesn't handle expected<> which means we don't have to invoke
                    // it if the previous task error'd out or its cancellation token is set.
                    if (input.has_error())
                        return typename as_expected<callable_return_t>::type{ input.error() };

                    if (cancel.cancelled())
                        return typename as_expected<callable_return_t>::type{ std::make_error_code(std::errc::operation_canceled) };

                    return output_wrapper<callable_return_t>::invoke(callable, input.value());
                };
            }
        };

        template<>
        struct input_output_wrapper<void>
        {
            template<typename CallableT>
            static auto wrap_callable(CallableT&& callable, cancellation& cancel)
            {
                return wrap_input(std::forward<CallableT>(callable), cancel,
                    decltype(handles_expected(instance_of<CallableT>())){});
            }

            template<typename CallableT>
            static auto wrap_input(CallableT&& callable, cancellation& cancel, std::true_type /*handles expected*/)
            {
                using callable_return_t = decltype(callable(mira::expected<void>{}));

                return[callable = std::forward<CallableT>(callable), &cancel](const expected<void>& input) mutable
                {
                    return invoke_callable_that_handles_expected<callable_return_t>(callable, cancel, input);
                };
            }

            template<typename CallableT>
            static auto wrap_input(CallableT&& callable, cancellation& cancel, std::false_type /*handles expected*/)
            {
                using callable_return_t = decltype(callable());

                return[callable = std::forward<CallableT>(callable), &cancel](const expected<void>& input) mutable
                {
                    // Here the callable doesn't handle expected<> which means we don't have to invoke
                    // it if the previous task error'd out or its cancellation token is set.
                    if (input.has_error())
                        return typename as_expected<callable_return_t>::type{ input.error() };

                    if (cancel.cancelled())
                        return typename as_expected<callable_return_t>::type{ std::make_error_code(std::errc::operation_canceled) };

                    return output_wrapper<callable_return_t>::invoke(callable);
                };
            }
        };

        template<size_t IndexV, typename ExpectedT, typename... TupleT>
        void write_expected_to_tuple(std::tuple<TupleT...>& tuple, const mira::expected<ExpectedT>& expected)
        {
            if (expected)
            {
                std::get<IndexV>(tuple) = expected.value();
            }
        }

        template<size_t IndexV, typename... TupleT>
        void write_expected_to_tuple(std::tuple<TupleT...>&, const mira::expected<void>&)
        {
        }
    }
}
