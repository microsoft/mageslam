//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#ifdef __cpp_coroutines

#include <CppUnitTest.h>

#include <arcana/threading/dispatcher.h>
#include <arcana/threading/coroutine.h>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(CoroutineTests)
    {
    public:
        TEST_METHOD(VoidTaskReturningCoroutine_GeneratesTask)
        {
            bool executed = false;

            auto coroutine = [&executed]() -> mira::task<void>
            {
                executed = true;
                co_return mira::coroutine_success;
            };

            auto task = coroutine();

            Assert::IsTrue(executed, L"Coroutine did not execute");
        }

        TEST_METHOD(ValueTaskReturningCoroutine_GeneratesTask)
        {
            auto coroutine = []() -> mira::task<int>
            {
                co_return 42;
            };

            auto task = coroutine();

            bool completed = false;
            task.then(mira::inline_scheduler, mira::cancellation::none(), [&completed](int value)
            {
                completed = true;
                Assert::AreEqual(42, value, L"Task does not have the expected value");
            });

            Assert::IsTrue(completed, L"Task did not complete synchronously");
        }

        TEST_METHOD(ValueTaskReturningCoroutine_CanReturnErrorCode)
        {
            auto error = std::make_error_code(std::errc::operation_not_supported);

            auto coroutine = [&error]() -> mira::task<int>
            {
                co_return error;
            };

            auto task = coroutine();

            bool completed = false;
            task.then(mira::inline_scheduler, mira::cancellation::none(), [&completed, &error](const mira::expected<int>& result)
            {
                completed = true;
                Assert::AreEqual(error.value(), result.error().value(), L"Task does not have the expected error state");
            });

            Assert::IsTrue(completed, L"Task did not complete synchronously");
        }

        TEST_METHOD(CoAwaitingVoidReturningTask_ResumesOnCompletion)
        {
            bool executed = false;

            auto coroutine = [&executed]() -> std::future<void>
            {
                mira::expected<void> result = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_result());
                executed = true;
            };

            coroutine().wait();

            Assert::IsTrue(executed, L"Continuation did not execute");
        }

        TEST_METHOD(CoAwaitingValueReturningTask_ResumesOnCompletion)
        {
            auto coroutine = []() -> std::future<int>
            {
                mira::expected<int> result = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_result(42));
                co_return result.value();
            };

            auto future = coroutine();

            Assert::AreEqual(42, future.get(), L"Task does not have the expected value");
        }

        TEST_METHOD(CoAwaitingValueReturningTask_CanReturnValue)
        {
            auto coroutine = []() -> std::future<int>
            {
                int result = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_result(42));
                co_return result;
            };

            auto future = coroutine();

            Assert::AreEqual(42, future.get(), L"Task does not have the expected value");
        }

        TEST_METHOD(ValueTaskReturningCoroutine_CanCoAwaitTask)
        {
            auto coroutine = []() -> mira::task<int>
            {
                mira::expected<int> result = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_result(42));
                co_return result.value();
            };

            auto task = coroutine();

            bool completed = false;
            task.then(mira::inline_scheduler, mira::cancellation::none(), [&completed](int value)
            {
                completed = true;
                Assert::AreEqual(42, value, L"Task does not have the expected value");
            });

            Assert::IsTrue(completed, L"Task did not complete synchronously");
        }

        TEST_METHOD(CoAwaitingVoidReturningTask_CanPropagateError)
        {
            auto error = std::make_error_code(std::errc::operation_not_supported);

            auto coroutine = [&error]() -> mira::task<void>
            {
                co_await mira::configure_await(mira::inline_scheduler, mira::task_from_error<void>(error));
                co_return mira::coroutine_success;
            };

            auto task = coroutine();

            task.then(mira::inline_scheduler, mira::cancellation::none(), [&error](const mira::expected<void>& result)
            {
                Assert::AreEqual(error.value(), result.error().value(), L"Final task does not contain expected error");
            });
        }

        TEST_METHOD(CoAwaitingValueReturningTask_CanPropagateError)
        {
            auto error = std::make_error_code(std::errc::operation_not_supported);

            auto coroutine = [&error]() -> mira::task<int>
            {
                int result = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_error<int>(error));
                co_return result * 42;
            };

            auto task = coroutine();

            task.then(mira::inline_scheduler, mira::cancellation::none(), [&error](const mira::expected<int>& result)
            {
                Assert::AreEqual(error.value(), result.error().value(), L"Final task does not contain expected error");
            });
        }

        TEST_METHOD(CoAwaitingErrorReturningTask_CanManuallyHandleError)
        {
            auto error = std::make_error_code(std::errc::operation_not_supported);

            auto coroutine = [&error]() -> mira::task<int>
            {
                mira::expected<int> result = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_error<int>(error));
                co_return 42;
            };

            auto task = coroutine();

            task.then(mira::inline_scheduler, mira::cancellation::none(), [&error](const mira::expected<int>& result)
            {
                Assert::AreEqual(42, result.value(), L"Final task does not contain expected result");
            });
        }

        //TEST_METHOD(CoAwaitingValueReturningTask_CanPropagateErrorInception)
        //{
        //    auto coroutine = []() -> mira::task<int>
        //    {
        //        auto error = std::make_error_code(std::errc::not_supported);
        //        auto result1 = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_error<void>(error));
        //        auto result2 = co_await mira::configure_await(mira::inline_scheduler, mira::task_from_error<void>(error));
        //        co_return 42;
        //    };

        //    auto task = coroutine();

        //    // TODO: This should blow up, but it should blow up in a way that makes it easy to diagnose.
        //}

        TEST_METHOD(SwitchToScheduler_ChangesContext)
        {
            mira::background_dispatcher<32> background;

            auto coroutine = [&background]() -> std::future<void>
            {
                auto foregroundThreadId = std::this_thread::get_id();
                co_await switch_to(background);
                auto backgroundThreadId = std::this_thread::get_id();

                Assert::AreNotEqual(std::hash<std::thread::id>{}(foregroundThreadId), std::hash<std::thread::id>{}(backgroundThreadId));
            };

            coroutine().wait();
        }
    };
}
#endif
