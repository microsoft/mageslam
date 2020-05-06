#include <CppUnitTest.h>

#include <arcana/threading/dispatcher.h>

#include <arcana/threading/task.h>

#include <arcana/threading/pending_task_scope.h>

#include <arcana/expected.h>

#include <numeric>
#include <algorithm>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(TaskUnitTest)
    {
        TEST_METHOD(CancellationCallback)
        {
            mira::cancellation_source source;

            int hit = 0;
            auto rego = source.add_listener([&]
            {
                hit++;
            });

            Assert::AreEqual(0, hit);

            source.cancel();

            Assert::AreEqual(1, hit);
        }

        TEST_METHOD(TaskSimpleOrdering)
        {
            mira::manual_dispatcher<32> dis;

            std::stringstream ss;

            mira::make_task(dis, mira::cancellation::none(), [&]
            {
                ss << "A";
            }).then(dis, mira::cancellation::none(), [&]
            {
                ss << "B";
            }).then(dis, mira::cancellation::none(), [&]
            {
                ss << "C";
            });

            mira::cancellation_source cancel;
            while (dis.tick(cancel)) {};

            Assert::AreEqual<std::string>("ABC", ss.str());
        }

        TEST_METHOD(TransformTaskFromResult)
        {
            int result = 0;
            mira::task_from_result(10).then(mira::inline_scheduler, mira::cancellation::none(), [&](int value)
            {
                result = 2 * value;
            });

            Assert::AreEqual(20, result);
        }

        TEST_METHOD(CollapsedTaskOrdering)
        {
            auto one = mira::task_from_result(),
                two = mira::task_from_result();

            mira::task_completion_source<void> start;
            mira::task_completion_source<void> other;

            std::stringstream ss;

            auto starttask = start.as_task();

            auto composed = starttask.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "1";

                return one.then(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    ss << "2";

                    return other.as_task().then(mira::inline_scheduler, mira::cancellation::none(), [&]
                    {
                        return two;
                    });

                }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    ss << "4";
                });
            });

            other.as_task().then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "3";
            });

            two.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "0";
            });

            composed.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "5";
            });

            auto composed2 = composed.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "6";
            });

            // composed2 continuations should run before this extra composed continuation
            composed.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "8";
            });

            composed2.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ss << "7";
            });

            start.complete();
            other.complete();

            Assert::AreEqual<std::string>("012345678", ss.str());
        }

        TEST_METHOD(TaskDualOrdering)
        {
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::stringstream ss;

            make_task(dis1, mira::cancellation::none(), [&]
            {
                ss << "A";
            }).then(dis2, mira::cancellation::none(), [&]
            {
                ss << "B";
            }).then(dis1, mira::cancellation::none(), [&]
            {
                ss << "C";
            });

            mira::cancellation_source cancel;
            while (dis1.tick(cancel) || dis2.tick(cancel)) {};

            Assert::AreEqual<std::string>("ABC", ss.str());
        }

        TEST_METHOD(TaskInvertedDualOrdering)
        {
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::stringstream ss;

            make_task(dis1, mira::cancellation::none(), [&]
            {
                ss << "A";
            }).then(dis2, mira::cancellation::none(), [&]
            {
                ss << "B";
            }).then(dis1, mira::cancellation::none(), [&]
            {
                ss << "C";
            });

            mira::cancellation_source cancel;
            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual<std::string>("ABC", ss.str());
        }

        TEST_METHOD(TaskThreadedOrdering)
        {
            mira::background_dispatcher<32> dis1;
            mira::background_dispatcher<32> dis2;
            mira::background_dispatcher<32> dis3;

            std::promise<void> work;
            std::future<void> finished = work.get_future();

            std::stringstream ss;

            make_task(dis1, mira::cancellation::none(), [&]
            {
                ss << "A";
            }).then(dis2, mira::cancellation::none(), [&]
            {
                ss << "B";
            }).then(dis3, mira::cancellation::none(), [&]
            {
                ss << "C";
            }).then(dis1, mira::cancellation::none(), [&]
            {
                work.set_value();
            });

            finished.wait();

            Assert::AreEqual<std::string>("ABC", ss.str());
        }

        TEST_METHOD(TaskReturnValue)
        {
            mira::background_dispatcher<32> dis1;
            mira::background_dispatcher<32> dis2;

            std::promise<std::string> work;
            std::future<std::string> finished = work.get_future();

            make_task(dis1, mira::cancellation::none(), [&]() -> std::string
            {
                return "A";
            }).then(dis2, mira::cancellation::none(), [&](const std::string& letter)
            {
                return letter + "B";
            }).then(dis1, mira::cancellation::none(), [&](const std::string& letter)
            {
                return letter + "C";
            }).then(dis2, mira::cancellation::none(), [&](const std::string& result)
            {
                work.set_value(result);
            });

            Assert::AreEqual<std::string>("ABC", finished.get());
        }

        struct counter
        {
            counter() = default;

            counter(int& value)
                : m_value{ &value }
            {
                (*m_value) = 0;
            }

            counter(counter&& other)
                : m_value{other.m_value}
            {
                other.m_value = nullptr;
            }

            counter& operator=(counter&& other)
            {
                std::swap(m_value, other.m_value);
                return *this;
            }

            ~counter()
            {
                if (m_value)
                    (*m_value) += 1;
            }

            int* m_value = nullptr;
        };

        TEST_METHOD(TaskCleanupResults)
        {
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int destructions;

            make_task(dis1, mira::cancellation::none(), [&]()
            {
                return counter(destructions);
            }).then(dis2, mira::cancellation::none(), [&](const counter& count)
            {
                Assert::AreNotEqual<int*>(nullptr, count.m_value);
            }).then(dis2, mira::cancellation::none(), [&]
            {
                Assert::AreEqual(1, destructions);
            });

            mira::cancellation_source cancel;
            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual(1, destructions);
        }

        TEST_METHOD(TaskCleanupLambdas)
        {
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::shared_ptr<int> shared = std::make_shared<int>(10);
            std::weak_ptr<int> weak = shared;

            make_task(dis1, mira::cancellation::none(), [shared]()
            {
            }).then(dis2, mira::cancellation::none(), [shared]
            {
            }).then(dis2, mira::cancellation::none(), [shared]
            {
            });

            shared.reset();

            mira::cancellation_source cancel;
            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual<int*>(nullptr, weak.lock().get());
        }

        TEST_METHOD(TaskCleanupVoidLambdasAfterCancellation)
        {
            mira::cancellation_source cancel;
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::shared_ptr<int> shared = std::make_shared<int>(10);
            std::weak_ptr<int> weak = shared;

            int run = 0;

            {
                make_task(dis1, mira::cancellation::none(), [&, shared]
                {
                    run++;
                }).then(dis2, mira::cancellation::none(), [&, shared]
                {
                    run++;
                }).then(dis2, mira::cancellation::none(), [&, shared]
                {
                    run++;
                });
            }

            shared.reset();

            dis1.tick(cancel);

            dis1.clear();
            dis2.clear();

            Assert::AreEqual(1, run);
            Assert::AreEqual<int*>(nullptr, weak.lock().get());
        }

        TEST_METHOD(TaskCleanupLambdasAfterCancellation)
        {
            mira::cancellation_source cancel;
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::shared_ptr<int> shared = std::make_shared<int>(10);
            std::weak_ptr<int> weak = shared;

            int run = 0;

            {
                make_task(dis1, mira::cancellation::none(), [&, shared]
                {
                    run++;
                    return 0;
                }).then(dis2, mira::cancellation::none(), [&, shared](int value)
                {
                    run++;
                    return value + 1;
                }).then(dis2, mira::cancellation::none(), [&, shared](int value)
                {
                    run++;
                    return value + 1;
                });
            }

            shared.reset();

            dis1.tick(cancel);

            dis1.clear();
            dis2.clear();

            Assert::AreEqual(1, run);
            Assert::AreEqual<int*>(nullptr, weak.lock().get());
        }

        TEST_METHOD(LateContinuation)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::stringstream ss;

            auto task = make_task(dis1, mira::cancellation::none(), [&]
            {
                ss << "A";
            });

            dis1.tick(cancel);

            task = task.then(dis2, mira::cancellation::none(), [&]
            {
                ss << "B";
            });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            task.then(dis1, mira::cancellation::none(), [&]
            {
                ss << "C";
            });

            dis1.tick(cancel);

            Assert::AreEqual<std::string>("ABC", ss.str());
        }

        TEST_METHOD(FromResult)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::string result;

            mira::task_from_result<std::string>("A").then(dis2, mira::cancellation::none(), [&](const std::string& letter)
            {
                return letter + "B";
            }).then(dis1, mira::cancellation::none(), [&](const std::string& letter)
            {
                return letter + "C";
            }).then(dis2, mira::cancellation::none(), [&](const std::string& res) { result = res; });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual<std::string>("ABC", result);
        }

        TEST_METHOD(TaskReturningTask)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::shared_ptr<int> shared = std::make_shared<int>(10);
            std::weak_ptr<int> weak = shared;

            std::string result;

            mira::task_from_result<std::string>("A")
                .then(dis2, mira::cancellation::none(), [&, shared](const std::string& letter)
                {
                    return make_task(dis1, mira::cancellation::none(), [letter = letter + "B", shared]
                    {
                        return letter + "C";
                    });
                })
                .then(dis2, mira::cancellation::none(), [&, shared](const std::string& res)
                {
                    result = res;
                });

            shared.reset();

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual<std::string>("ABC", result);
            Assert::AreEqual(true, weak.expired());
        }

        TEST_METHOD(DifferentSizedTasks)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::shared_ptr<int> shared = std::make_shared<int>(10);
            std::weak_ptr<int> weak = shared;

            std::string result;

            std::array<char, 30> large;
            large.back() = 'B';

            mira::task_from_result<std::string>("A")
                .then(dis2, mira::cancellation::none(), [&, shared](const std::string& letter)
                {
                    return make_task(dis1, mira::cancellation::none(), [letter, large, shared]
                    {
                        return letter + large.back();
                    });
                }).then(dis2, mira::cancellation::none(), [&, letter = std::string{ "C" }](const std::string& res)
                {
                    result = res + letter;
                });

            shared.reset();

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual<std::string>("ABC", result);
            Assert::AreEqual(true, weak.expired());
        }

        TEST_METHOD(InlineContinuation)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;

            int runs = 0;
            auto task = make_task(dis1, mira::cancellation::none(), [&]
            {
                runs++;
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                runs++;
            });

            dis1.tick(cancel);

            Assert::AreEqual(2, runs);
        }

        TEST_METHOD(WhenAll)
        {
            mira::background_dispatcher<32> dis1;
            mira::background_dispatcher<32> dis2;

            std::promise<int> work;

            std::stringstream ss;

            int a = 0, b = 0, c = 0;

            std::vector<mira::task<void>> tasks;

            tasks.push_back(make_task(dis1, mira::cancellation::none(), [&]
            {
                c = 3;
            }));
            tasks.push_back(make_task(dis2, mira::cancellation::none(), [&]
            {
                b = 2;
            }));
            tasks.push_back(make_task(dis1, mira::cancellation::none(), [&]
            {
                a = 1;
            }));

            mira::when_all(gsl::make_span(tasks)).then(dis1, mira::cancellation::none(), [&]
            {
                work.set_value(a + b + c);
            });

            Assert::AreEqual(6, work.get_future().get());
        }

        TEST_METHOD(EmptyWhenAll)
        {
            mira::background_dispatcher<32> dis1;

            std::promise<int> work;

            mira::when_all(gsl::span<mira::task<void>>{}).then(dis1, mira::cancellation::none(), [&]
            {
                work.set_value(6);
            });

            Assert::AreEqual(6, work.get_future().get());
        }

        TEST_METHOD(WhenAllVariadicWithVoid)
        {
            mira::background_dispatcher<32> dis1;
            std::promise<int> work;

            mira::task<void> t1 = make_task(dis1, mira::cancellation::none(), []()
            {
            });

            mira::task<int> t2 = make_task(dis1, mira::cancellation::none(), []()
            {
                return 5;
            });

            mira::task<void> t3 = make_task(dis1, mira::cancellation::none(), []()
            {
            });

            mira::when_all(t1, t2, t3).then(dis1, mira::cancellation::none(), [&](const std::tuple<mira::void_placeholder, int, mira::void_placeholder>& args)
            {
                work.set_value(std::get<1>(args));
            });

            Assert::AreEqual(5, work.get_future().get());
        }

        TEST_METHOD(SynchronousPendingTaskScope)
        {
            mira::pending_task_scope scope;

            scope += mira::task_from_result();

            Assert::IsTrue(scope.completed());
        }

        TEST_METHOD(SynchronousPendingTaskScope_WhenAll)
        {
            mira::pending_task_scope scope;

            scope += mira::task_from_result();

            bool didRun = false;
            scope.when_all().then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                didRun = true;
            });

            Assert::IsTrue(scope.completed());
            Assert::IsTrue(didRun);
        }

        TEST_METHOD(PendingTaskScopeCompletionOrder)
        {
            mira::pending_task_scope scope;

            mira::manual_dispatcher<32> dis1;

            int result = 0;
            auto work = make_task(dis1, mira::cancellation::none(), [&]
            {
                result = 10;
            });

            work.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return scope.when_all().then(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    Assert::IsTrue(scope.completed(), L"a continuation on a scope when_all should guarantee that the scope is done");
                });
            });

            scope += work;

            while (dis1.tick(mira::cancellation::none())) {};

            Assert::IsTrue(scope.completed());
        }

        TEST_METHOD(PendingTaskScopeBubbleError)
        {
            mira::pending_task_scope scope;
            const auto error = std::make_error_code(std::errc::owner_dead);
            scope += mira::task_from_error<void>(error);
            Assert::IsTrue(scope.completed());
            Assert::IsTrue(scope.has_error());
            Assert::IsTrue(scope.error() == error);

            bool taskComplete = false;
            const auto task = scope.when_all().then(mira::inline_scheduler, mira::cancellation::none(), [&](mira::expected<void> previous)
            {
                taskComplete = true;
                Assert::IsTrue(previous.has_error());
                Assert::IsTrue(previous.error() == error);
            });

            Assert::IsTrue(taskComplete);
        }

        TEST_METHOD(LastMethodAlwaysRuns)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int result = -1;

            bool wasCalled = false;

            mira::task_from_result(10)
                .then(dis2, mira::cancellation::none(), [&](int value)
                {
                    return value;
                }).then(dis2, mira::cancellation::none(), [&](int /*value*/) -> mira::expected<int>
                {
                    return std::errc::operation_canceled;
                }).then(dis1, mira::cancellation::none(), [&](int /*value*/)
                {
                    wasCalled = true;
                }).then(dis2, mira::cancellation::none(), [&](const mira::expected<void>& value)
                {
                    Assert::IsTrue(value.has_error());
                    result = 15;
                });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::IsFalse(wasCalled, L"This method shouldn't run because it doesn't care about errors");
            Assert::AreEqual(15, result);
        }

        TEST_METHOD(AutomaticCancellation)
        {
            mira::cancellation_source cancel;
            mira::cancellation_source global;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int hitCount = 0;
            bool wasCalled1 = false, wasCalled2 = false;

            mira::task_from_result(10)
                .then(dis2, cancel, [&](int value)
                {
                    hitCount++;
                    return 2 * value;
                }).then(dis2, cancel, [&](int value)
                {
                    hitCount++;
                    wasCalled1 = true;
                    return value + 5;
                }).then(dis1, cancel, [&](int /*value*/)
                {
                    hitCount++;
                    wasCalled2 = true;
                }).then(dis2, mira::cancellation::none(), [&](const mira::expected<void>& value)
                {
                    hitCount++;
                    Assert::IsTrue(value.has_error() && value.error() == std::errc::operation_canceled);
                });

            dis2.tick(global);

            cancel.cancel();

            while (dis2.tick(global) || dis1.tick(global)) {};

            Assert::IsFalse(wasCalled1, L"This method shouldn't run");
            Assert::IsFalse(wasCalled2, L"This method shouldn't run");
            Assert::AreEqual(2, hitCount);
        }

        TEST_METHOD(CancellationOrder_IsReverseOfOrderAdded)
        {
            mira::cancellation_source root;

            int hitCount = 0;

            auto tick1 = root.add_listener([&]
            {
                Assert::AreEqual(1, hitCount);
                hitCount++;
            });

            auto tick2 = root.add_listener([&]
            {
                Assert::AreEqual(0, hitCount);
                hitCount++;
            });

            root.cancel();

            Assert::AreEqual(2, hitCount);
        }

        TEST_METHOD(IfErrorThenCancelationReturnError)
        {
            mira::cancellation_source cancel;
            mira::cancellation_source global;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int hitCount = 0;
            bool wasCalled1 = false, wasCalled2 = false;

            mira::task_from_result(10)
                .then(dis2, cancel, [&](int /*value*/)
            {
                hitCount++;
                return mira::expected<int>{ std::errc::bad_message };
            }).then(dis2, cancel, [&](int value)
            {
                hitCount++;
                wasCalled1 = true;
                return value + 5;
            }).then(dis1, cancel, [&](int /*value*/)
            {
                hitCount++;
                wasCalled2 = true;
            }).then(dis2, mira::cancellation::none(), [&](const mira::expected<void>& value)
            {
                hitCount++;
                Assert::IsTrue(value.has_error() && value.error() == std::errc::bad_message);
            });

            dis2.tick(global);

            cancel.cancel();

            while (dis2.tick(global) || dis1.tick(global)) {};

            Assert::IsFalse(wasCalled1, L"This method shouldn't run");
            Assert::IsFalse(wasCalled2, L"This method shouldn't run");
            Assert::AreEqual(2, hitCount);
        }

        TEST_METHOD(ExpectedToValueConversion)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int result = -1;

            mira::task_from_result(10)
                .then(dis2, mira::cancellation::none(), [&](int value)
                {
                    return value;
                }).then(dis2, mira::cancellation::none(), [&](int value) -> mira::expected<int>
                {
                    return value;
                }).then(dis2, mira::cancellation::none(), [&](const mira::expected<int>& value) -> mira::expected<int>
                {
                    return value.value() + 5;
                }).then(dis1, mira::cancellation::none(), [&](int value)
                {
                    result = value;
                });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::AreEqual(15, result);
        }

        TEST_METHOD(ErrorCodeTasks)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            mira::expected<int> result;

            mira::task_from_result(10)
                .then(dis2, mira::cancellation::none(), [&](int value) -> mira::task<int>
                {
                    return make_task(dis1, mira::cancellation::none(), [value]() -> mira::expected<int>
                    {
                        return value + 1;
                    });
                }).then(dis2, mira::cancellation::none(), [&](const mira::expected<int>& value) -> mira::expected<int>
                {
                    if (!value)
                        return -1;

                    return 10;
                }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& value)
                {
                    result = value;
                });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::IsTrue(result.has_value());
            Assert::AreEqual(10, result.value());

            mira::task_from_result(10)
                .then(dis2, mira::cancellation::none(), [&](int value)
            {
                return make_task(dis1, mira::cancellation::none(), [value]() -> mira::expected<int>
                {
                    return value + 1;
                });
            }).then(dis2, mira::cancellation::none(), [&](const mira::expected<int>& value) -> mira::expected<int>
            {
                if (!value || value.value() > 10)
                    return std::errc::invalid_argument;

                return value;
            }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& value)
            {
                result = value;
            });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::IsFalse(result.has_value());
            Assert::IsTrue(result.error() == std::errc::invalid_argument);
        }

        TEST_METHOD(ChainingTasksAndExpecteds)
        {
            mira::cancellation_source cancel;
            mira::manual_dispatcher<32> dis1;

            int hit = 0;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4702) // Unreachable code
#endif
            mira::task_from_result(std::make_shared<const int>(10))
                .then(dis1, mira::cancellation::none(), [&](const std::shared_ptr<const int>& i)
                {
                    hit++;
                    return make_task(dis1, mira::cancellation::none(), [&, i]() -> mira::expected<std::shared_ptr<const double>>
                    {
                        hit++;
                        return std::errc::operation_canceled;
                    });
            }).then(dis1, mira::cancellation::none(), [&](const std::shared_ptr<const double>&)
            {
                hit++;
                Assert::Fail(L"This should not have run");
                return 10;
            }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& something)
            {
                hit++;
                Assert::IsTrue(std::errc::operation_canceled == something.error());
            });
#ifdef _MSC_VER
#pragma warning(pop)
#endif

            while (dis1.tick(cancel)) {};

            Assert::AreEqual(3, hit);
        }

        TEST_METHOD(ChainingTasksAndConvertingToExpecteds)
        {
            mira::cancellation_source cancel;
            mira::manual_dispatcher<32> dis1;

            int hit = 0;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4702) // Unreachable code
#endif
            mira::task_from_result(std::make_shared<const int>(10))
                .then(dis1, mira::cancellation::none(), [&](const std::shared_ptr<const int>& i)
            {
                hit++;
                return make_task(dis1, mira::cancellation::none(), [&, i]() -> mira::expected<std::shared_ptr<const double>>
                {
                    hit++;
                    return std::errc::operation_canceled;
                });
            }).then(dis1, mira::cancellation::none(), [&](const std::shared_ptr<const double>&)
            {
                hit++;
                Assert::Fail(L"This should not have run");
                return 10;
            }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& something)
            {
                hit++;
                Assert::IsTrue(std::errc::operation_canceled == something.error());
            });
#ifdef _MSC_VER
#pragma warning(pop)
#endif

            while (dis1.tick(cancel)) {};

            Assert::AreEqual(3, hit);
        }

        TEST_METHOD(ChainingTasksAndTryingToGetAroundExpecteds)
        {
            mira::cancellation_source cancel;
            mira::manual_dispatcher<32> dis1;

            int hit = 0;

            mira::task_from_error<std::shared_ptr<const int>>(std::errc::operation_canceled)
                .then(dis1, mira::cancellation::none(), [&](const mira::expected<std::shared_ptr<const int>>& i)
                {
                    hit++;

                    // Here we ignore the prior error, and we just schedule a task
                    // which means all other tasks are going to execute as normal.
                    // This is kindof a gotcha, but if you're taking an expected it's
                    // your job to propagate it.
                    return make_task(dis1, mira::cancellation::none(), [&, i]() -> std::shared_ptr<const double>
                    {
                        hit++;
                        return std::make_shared<const double>();
                    });
                }).then(dis1, mira::cancellation::none(), [&](const std::shared_ptr<const double>&)
                {
                    hit++;

                    return 10;
                }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& something)
                {
                    hit++;
                    Assert::IsTrue(something.has_value());
                });

            while (dis1.tick(cancel)) {};

            Assert::AreEqual(4, hit);
        }

        TEST_METHOD(ChainingTasksAndExpectedsOnError)
        {
            mira::cancellation_source cancel;
            mira::manual_dispatcher<32> dis1;

            int hit = 0;

            mira::task_from_error<std::shared_ptr<const int>>(std::errc::operation_canceled)
                .then(dis1, mira::cancellation::none(), [&](const mira::expected<std::shared_ptr<const int>>& i)
                {
                    hit++;

                    if (i.has_error())
                    {
                        return mira::task_from_result(mira::expected<std::shared_ptr<const double>>{i.error()});
                    }

                    return make_task(dis1, mira::cancellation::none(), [&, i]() -> std::shared_ptr<const double>
                    {
                        hit++;
                        return std::make_shared<const double>();
                    });
                }).then(dis1, mira::cancellation::none(), [&](const std::shared_ptr<const double>&)
                {
                    hit++;

                    return 10;
                }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& something)
                {
                    hit++;
                    Assert::IsTrue(something.has_error());
                });

            while (dis1.tick(cancel)) {};

            Assert::AreEqual(2, hit);
        }

        TEST_METHOD(CancellingTasks)
        {
            mira::cancellation_source cancel;

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            mira::expected<int> result;

            mira::task_from_result(10)
                .then(dis2, mira::cancellation::none(), [&](int value)
            {
                return make_task(dis1, mira::cancellation::none(), [value]() -> mira::expected<int>
                {
                    return value + 1;
                });
            }).then(dis2, mira::cancellation::none(), [&](const mira::expected<int>& /*value*/) -> mira::expected<int>
            {
                return 10;
            }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& value)
            {
                result = value;
            });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::IsTrue(result.has_value());
            Assert::AreEqual(10, result.value());

            mira::task_from_result(10)
                .then(dis2, mira::cancellation::none(), [&](int value)
            {
                return make_task(dis1, mira::cancellation::none(), [value]() -> mira::expected<int>
                {
                    return value + 1;
                });
            }).then(dis2, mira::cancellation::none(), [&](const mira::expected<int>& value) -> mira::expected<int>
            {
                if (!value || value.value() > 10)
                    return std::errc::invalid_argument;

                return value;
            }).then(dis1, mira::cancellation::none(), [&](const mira::expected<int>& value) -> mira::expected<void>
            {
                result = value;

                return {};
            });

            while (dis2.tick(cancel) || dis1.tick(cancel)) {};

            Assert::IsFalse(result.has_value());
            Assert::IsTrue(result.error() == std::errc::invalid_argument);
        }

        TEST_METHOD(WhenAllResults)
        {
            // lets do 16 / 8

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int result{};

            auto sixteen = mira::task_from_result(16);
            auto eight = mira::task_from_result(8);

            when_all(sixteen, eight).then(dis1, mira::cancellation::none(), [](const std::tuple<int, int>& values)
            {
                return std::get<0>(values) / std::get<1>(values);
            }).then(dis2, mira::cancellation::none(), [&](int value)
            {
                result = value;
            });

            while (dis2.tick(mira::cancellation::none()) || dis1.tick(mira::cancellation::none())) {};

            Assert::AreEqual(2, result);
        }

        TEST_METHOD(MultipleWhenAlls)
        {
            // lets do 10 * (4 + 16 / 8)

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int result{};

            auto four = mira::task_from_result(4);
            auto sixteen = mira::task_from_result(16);
            auto eight = mira::task_from_result(8);

            auto div = when_all(sixteen, eight).then(dis2, mira::cancellation::none(), [](const std::tuple<int, int>& value)
            {
                return std::get<0>(value) / std::get<1>(value);
            });

            auto sum = when_all(four, div).then(dis1, mira::cancellation::none(), [](const std::tuple<int, int>& value)
            {
                return std::get<0>(value) + std::get<1>(value);
            });

            auto mul = mira::when_all(mira::task_from_result(10), sum).then(dis1, mira::cancellation::none(), [](const std::tuple<int, int>& value)
            {
                return std::get<0>(value) * std::get<1>(value);
            });

            mul.then(dis2, mira::cancellation::none(), [&](int value)
            {
                result = value;
            });

            while (dis2.tick(mira::cancellation::none()) || dis1.tick(mira::cancellation::none())) {};

            Assert::AreEqual(60, result);
        }

        TEST_METHOD(DifferentWhenAllTypes)
        {
            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            std::string result;

            auto repetitions = mira::task_from_result(3);
            auto word = mira::task_from_result<std::string>("Snaaaaaaake");

            auto div = when_all(repetitions, word).then(dis2, mira::cancellation::none(), [](const std::tuple<int, std::string>& value)
            {
                std::stringstream stream{};
                for (int i = 0; i < std::get<0>(value); ++i)
                {
                    stream << std::get<1>(value);
                }
                return stream.str();
            }).then(dis1, mira::cancellation::none(), [&](const std::string& value)
            {
                result = value;
            });

            while (dis2.tick(mira::cancellation::none()) || dis1.tick(mira::cancellation::none())) {};

            Assert::AreEqual<std::string>("SnaaaaaaakeSnaaaaaaakeSnaaaaaaake", result);
        }

        TEST_METHOD(WhenAll_StringVector)
        {
            mira::manual_dispatcher<32> dis;

            std::string result = "";
            std::vector<mira::task<std::string>> tasks
            {
                mira::task_from_result<std::string>("H"),
                mira::task_from_result<std::string>("e"),
                mira::task_from_result<std::string>("l"),
                mira::task_from_result<std::string>("l"),
                mira::task_from_result<std::string>("o")
            };


            mira::when_all(gsl::make_span(tasks))
                .then(dis, mira::cancellation::none(), [&](std::vector<std::string> results)
            {
                for (auto i = 0U; i < results.size(); ++i)
                {
                    result += results[i];
                }
                return result;
            });

            while (dis.tick(mira::cancellation::none())) {};
            Assert::AreEqual<std::string>("Hello", result);
        }

        TEST_METHOD(WhenAll_MathOperations)
        {
            // lets do 10 * (4 + 16 / 8) as the previous tests

            mira::manual_dispatcher<32> dis1;
            mira::manual_dispatcher<32> dis2;

            int result{};

            std::vector<mira::task<int>> divisors = { mira::task_from_result(16), mira::task_from_result(8) };
            std::vector<mira::task<int>> addition = { mira::task_from_result(4) };
            std::vector<mira::task<int>> multipliers = { mira::task_from_result(10) };

            auto div = when_all(gsl::make_span(divisors)).then(dis2, mira::cancellation::none(), [](const std::vector<int>& value)
            {
                return value.at(0) / value.at(1);
            });
            addition.push_back(div); //append answer

            auto sum = when_all(gsl::make_span(addition)).then(dis1, mira::cancellation::none(), [](const std::vector<int>& value)
            {
                return value.at(0) + value.at(1);
            });
            multipliers.push_back(sum); //append answer

            auto mul = mira::when_all(gsl::make_span(multipliers)).then(dis1, mira::cancellation::none(), [](const std::vector<int>& value)
            {
                return value.at(0) * value.at(1);
            });

            mul.then(dis2, mira::cancellation::none(), [&](int value)
            {
                result = value;
            });

            while (dis2.tick(mira::cancellation::none()) || dis1.tick(mira::cancellation::none())) {};
            Assert::AreEqual(60, result);
        }

        TEST_METHOD(WhenAll_BooleanValues)
        {
            mira::manual_dispatcher<32> dis;

            bool result;
            std::vector<mira::task<bool>> tasks
            {
                mira::task_from_result(true),
                mira::task_from_result(true),
                mira::task_from_result(true),
                mira::task_from_result(true)
            };

            mira::when_all(gsl::make_span(tasks))
                .then(dis, mira::cancellation::none(), [&](std::vector<bool> results)
            {
                result = std::all_of(begin(results), end(results), [](const bool& val){ return val; });
            });

            while (dis.tick(mira::cancellation::none())) {};
            Assert::AreEqual<bool>(true, result);

            tasks.push_back(mira::task_from_result(false));

            mira::when_all(gsl::make_span(tasks))
                .then(dis, mira::cancellation::none(), [&](std::vector<bool> results)
            {
                result = std::all_of(begin(results), end(results), [](const bool& val){ return val; });
            });

            while (dis.tick(mira::cancellation::none())) {};
            Assert::AreEqual<bool>(false, result);
        }

        // this function uses most of the features of the task system.
        // - it tests multiple continuations by making each task consumed in
        // the two next tasks.
        // - tests out that when_all converts to the right tuple types
        // - matches completed tasks with non-completed tasks
        mira::task<int> fibonnaci(mira::dispatcher<32>& dis, int n)
        {
            std::vector<mira::task<int>> fibtasks{ mira::task_from_result(0), mira::task_from_result(1) };

            for (int i = 2; i <= n; ++i)
            {
                auto nextfib = mira::when_all(*(fibtasks.rbegin() + 1), *fibtasks.rbegin())
                    .then(dis, mira::cancellation::none(), [](const std::tuple<int, int>& other)
                    {
                        return std::get<0>(other) + std::get<1>(other);
                    });

                fibtasks.emplace_back(std::move(nextfib));
            }

            return fibtasks[n];
        }

        TEST_METHOD(MultipleContinuationFibonnaci)
        {
            mira::manual_dispatcher<32> dis1;

            mira::task<int> myfib = fibonnaci(dis1, 42);

            int result{};
            myfib.then(dis1, mira::cancellation::none(), [&](int r)
            {
                result = r;
            });

            while (dis1.tick(mira::cancellation::none())) {};

            Assert::AreEqual(267914296, result);
        }

        TEST_METHOD(CancellationStackBuster)
        {
            mira::task_completion_source<void> signal{};

            mira::cancellation_source cancellation;

            std::vector<int> depth;

            mira::task<int> parent = signal.as_task().then(mira::inline_scheduler, cancellation, []
            {
                return -1;
            });

            for (int d = 0; d < 200; ++d)
            {
                parent = parent.then(mira::inline_scheduler, cancellation, [&, d](int /*old*/)
                {
                    depth.push_back(d);

                    return d;
                });
            }

            cancellation.cancel();

            signal.complete();
        }

        static mira::task<void> CreateNestedTaskChain(size_t depth)
        {
            if (depth == 0)
                return mira::task_from_result();

            return mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [depth]
            {
                return CreateNestedTaskChain(depth - 1);
            });
        }

        TEST_METHOD(LargeNestedSetOfStasks)
        {
            mira::task_completion_source<void> signal{};

            std::vector<int> depth;

            mira::task<void> parent = signal.as_task().then(mira::inline_scheduler, mira::cancellation::none(), []
            {
                return CreateNestedTaskChain(200);
            });

            bool completed = false;
            parent.then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                completed = true;
            });

            signal.complete();

            Assert::IsTrue(completed);
        }

        // This test validates that continuations are properly
        // reparented when unwrapped. When not properly reparented,
        // parents would get destroyed because they were old
        // task_completion_sources and when executed the lock on
        // the weak_ptr would fail.
        TEST_METHOD(CompletionSourceOfCompletionSource)
        {
            mira::task_completion_source<void> source;

            mira::expected<void> result;

            mira::manual_dispatcher<32> background;

            {
                mira::make_task(background, mira::cancellation::none(), [&]
                {
                    return source.as_task();
                }).then(background, mira::cancellation::none(), [&](const mira::expected<void>& r)
                {
                    result = r;
                });
            }

            background.tick(mira::cancellation::none());

            source.complete(std::errc::operation_canceled);

            background.tick(mira::cancellation::none());

            Assert::IsTrue(result.has_error());
            Assert::IsTrue(result.error() == std::errc::operation_canceled);
        }

        mira::task<void> RunTaskAsGenerator(mira::manual_dispatcher<32>& background, mira::cancellation& cancel, int& iterations)
        {
            return mira::make_task(background, cancel, [&]()
            {
                iterations++;

                return CreateNestedTaskChain(10).then(mira::inline_scheduler, mira::cancellation::none(), [&]()
                {
                    return RunTaskAsGenerator(background, cancel, iterations);
                });
            });
        }

        TEST_METHOD(GenerateLotsOfTasksRecursively)
        {
            bool completed = false;

            mira::manual_dispatcher<32> background;

            mira::cancellation_source cancel;

            mira::expected<void> result;

            int iterations = 0;

            {
                mira::task<void> parent = RunTaskAsGenerator(background, cancel, iterations);

                parent.then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>& r)
                {
                    completed = true;
                    result = r;
                });
            }

            int count = 1000;
            while (count > 0)
            {
                count--;
                background.tick(mira::cancellation::none());
            }

            cancel.cancel();

            while (background.tick(mira::cancellation::none()));

            Assert::IsTrue(completed, L"The chain hasn't completed properly");
            Assert::AreEqual(1000, iterations, L"The chain hasn't completed properly");
            Assert::IsTrue(result.has_error(), L"The result should have completed through cancellation");
            Assert::IsTrue(result.error() == std::errc::operation_canceled, L"The result should have completed through cancellation");
        }

        TEST_METHOD(NestedTaskChain)
        {
            auto task = CreateNestedTaskChain(1);

            bool completed = false;

            task.then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>&)
            {
                completed = true;
            });

            Assert::IsTrue(completed, L"The chain hasn't completed properly");
        }
    };
}
