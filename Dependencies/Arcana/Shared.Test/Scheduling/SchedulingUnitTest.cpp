#include <CppUnitTest.h>

#include <arcana\scheduling\state_machine.h>

#include <arcana\threading\dispatcher.h>
#include <arcana\threading\pending_task_scope.h>

#include <arcana\messaging\mediator.h>

#include <memory>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    mira::state_machine_state<bool> TrackingInit{ "TrackingInit" };

    mira::state_machine_state<void> TrackingRead{ "TrackingRead" };
    mira::state_machine_state<void> TrackingWrite{ "TrackingWrite" };

    struct ImageReceived
    {};

    using Mediator = mira::mediator<mira::dispatcher<32>, ImageReceived>;

    struct Context
    {
        Mediator& Mediator;
        mira::state_machine_observer& StateMachine;
        mira::dispatcher<32>& Dispatcher;
    };

    struct InitializationWorker
    {
        InitializationWorker(Context& context)
            : m_context{ context }
        {
            m_registrations += m_context.Mediator.add_listener<ImageReceived>([this](auto i) { Run(i); });
        }

        void Run(const ImageReceived&)
        {
            m_pending += m_context.StateMachine.on(TrackingInit, m_context.Dispatcher, m_cancel, [&](bool& result)
            {
                Count++;

                if (Count > 3)
                {
                    m_registrations.clear();
                    result = true;
                }
            });
        }
        
        mira::task<void> ShutdownAsync()
        {
            m_cancel.cancel();
            return m_pending.when_all();
        }

        int Count{};

    private:
        Context& m_context;
        mira::ticket_scope m_registrations;
        mira::pending_task_scope m_pending;
        mira::cancellation_source m_cancel;
    };

    struct TrackingWorker
    {
        TrackingWorker(Context& context)
            : m_context{ context }
        {
            m_registrations += m_context.Mediator.add_listener<ImageReceived>([this](auto i) { Run(i); });
        }

        void Run(const ImageReceived&)
        {
            if (m_cancel.cancelled())
                return;

            m_previous = m_previous.then(m_context.Dispatcher, m_cancel, [&]
            {
                return m_context.StateMachine.on(TrackingRead, m_context.Dispatcher, m_cancel, [&]
                {
                    return Result + 10;
                });
            }).then(m_context.Dispatcher, m_cancel, [&](int value)
            {
                value += 30;

                return m_context.StateMachine.on(TrackingWrite, m_context.Dispatcher, m_cancel, [this, value]
                {
                    Result += value;
                    Iterations++;
                });
            });

            m_scope += m_previous;
        }

        mira::task<void> ShutdownAsync()
        {
            m_cancel.cancel();

            return m_scope.when_all();
        }

        int Iterations{};
        int Result{};

    private:
        mira::task<void> m_previous = mira::task_from_result(mira::expected<void>{});
        Context& m_context;
        mira::ticket_scope m_registrations;
        mira::pending_task_scope m_scope;
        mira::cancellation_source m_cancel;
    };

    mira::task<void> LinearSchedule(mira::state_machine_driver& driver, mira::dispatcher<32>& dispatcher)
    {
        return mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
        {
            // do a tracking read
            return driver.move_to(TrackingRead, mira::cancellation::none());
        }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
        {
            // then a tracking write
            return driver.move_to(TrackingWrite, mira::cancellation::none());
        }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
        {
            // then repeat the current schedule
            return LinearSchedule(driver, dispatcher);
        });
    }

    mira::task<void> InitializationSchedule(mira::state_machine_driver& driver, mira::dispatcher<32>& dispatcher)
    {
        return make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
        {
            return driver.move_to(TrackingInit, mira::cancellation::none());
        }).then(mira::inline_scheduler, mira::cancellation::none(), [&](bool initialized)
        {
            if (initialized)
            {
                return LinearSchedule(driver, dispatcher);
            }
            else
            {
                // then repeat the current schedule
                return InitializationSchedule(driver, dispatcher);
            }
        });
    }

    TEST_CLASS(SchedulingUnitTest)
    {
        TEST_METHOD(RepeatingLinearSchedule)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            mira::manual_dispatcher<32> dispatch;
            Mediator mediator{ dispatch };

            LinearSchedule(driver, dispatch);

            Context context{
                mediator,
                sched,
                dispatch
            };

            TrackingWorker worker{ context };

            do
            {
                if (worker.Iterations == 2)
                {
                    worker.ShutdownAsync();
                    break;
                }

                mediator.send<ImageReceived>({});
            } while (dispatch.tick(mira::cancellation::none()));

            while (dispatch.tick(mira::cancellation::none())) {};

            Assert::AreEqual(120, worker.Result);
        }

        TEST_METHOD(ConditionalSchedule)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            mira::manual_dispatcher<32> dispatch;
            Mediator mediator{ dispatch };

            InitializationSchedule(driver, dispatch);

            Context context{
                mediator,
                sched,
                dispatch
            };

            InitializationWorker init{ context };
            TrackingWorker worker{ context };

            do
            {
                if (worker.Iterations == 2)
                {
                    worker.ShutdownAsync();
                    break;
                }

                mediator.send<ImageReceived>({});
            } while (dispatch.tick(mira::cancellation::none()));

            while (dispatch.tick(mira::cancellation::none())) {};

            Assert::AreEqual(4, init.Count);
            Assert::AreEqual(2, worker.Iterations);
            Assert::AreEqual(120, worker.Result);
        }

        TEST_METHOD(SendDataFromWorker)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };

            mira::state_machine_state<bool> one{ "One" }, two{ "Two" }, three{ "Three" };

            mira::signal signal;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&](const bool& data)
            {
                Assert::IsFalse(data);
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&](const bool& data)
            {
                Assert::IsTrue(data);
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&](const bool& data)
            {
                Assert::IsFalse(data);
            });

            std::stringstream ss;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return sched.on(one, mira::inline_scheduler, mira::cancellation::none(), [&](bool&)
                {
                    ss << "one";
                });
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return sched.on(two, mira::inline_scheduler, mira::cancellation::none(), [&](bool& data)
                {
                    ss << "two";
                    data = true;
                });
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return sched.on(three, mira::inline_scheduler, mira::cancellation::none(), [&](bool&)
                {
                    ss << "three";
                });
            });

            Assert::AreEqual<std::string>("onetwothree", ss.str());
        }

        TEST_METHOD(MoveToEachState)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            
            mira::state_machine_state<void> one{ "One" }, two{ "Two" }, three{ "Three" };

            mira::signal signal;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            });

            std::stringstream ss;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return sched.on(one, mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    ss << "one";
                });
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return sched.on(two, mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    ss << "two";
                });
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return sched.on(three, mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    ss << "three";
                });
            });

            Assert::AreEqual<std::string>("onetwothree", ss.str());
        }

        TEST_METHOD(CancellationCancelsTheSchedulingMethod)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };

            mira::state_machine_state<void> one{ "One" }, two{ "Two" };

            mira::cancellation_source cancel;

            bool driverFinished = false;

            mira::make_task(mira::inline_scheduler, cancel, [&]
            {
                return driver.move_to(one, cancel);
            }).then(mira::inline_scheduler, cancel, [&]
            {
                return driver.move_to(two, cancel);
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>&)
            {
                driverFinished = true;
            });

            bool observerFinished = false;

            mira::make_task(mira::inline_scheduler, cancel, [&]
            {
                cancel.cancel();
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>&)
            {
                observerFinished = true;
            });

            Assert::IsTrue(driverFinished);
            Assert::IsTrue(observerFinished);
        }

        mira::task<void> WorkOn(
            std::stringstream& stream,
            mira::state_machine_observer& sched,
            mira::state_machine_state<void>& state,
            mira::cancellation& cancel,
            mira::dispatcher<32>& dispatcher)
        {
            return mira::make_task(dispatcher, cancel, [&]
            {
                return sched.on(state, dispatcher, cancel, [&]
                {
                    stream << state.name();
                });
            }).then(dispatcher, cancel, [&]
            {
                return WorkOn(stream, sched, state, cancel, dispatcher);
            });
        }

        TEST_METHOD(SequentialSchedule)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            mira::cancellation_source cancel;
            mira::background_dispatcher<32> background;

            mira::state_machine_state<void> one{ "1" }, two{ "2" }, three{ "3" };

            mira::signal signal;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                signal.set();
            });

            std::stringstream ss;

            WorkOn(ss, sched, one, cancel, background);
            WorkOn(ss, sched, two, cancel, background);
            WorkOn(ss, sched, three, cancel, background);

            signal.wait(mira::cancellation::none(), false);
            cancel.cancel();

            Assert::AreEqual<std::string>("123", ss.str());
        }

        TEST_METHOD(InvertSequentialSchedule)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            mira::cancellation_source cancel;
            mira::background_dispatcher<32> background;

            mira::state_machine_state<void> one{ "1" }, two{ "2" }, three{ "3" };

            mira::signal signal;

            std::stringstream ss;

            WorkOn(ss, sched, one, cancel, background);
            WorkOn(ss, sched, two, cancel, background);
            WorkOn(ss, sched, three, cancel, background);

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                signal.set();
            });

            signal.wait(mira::cancellation::none(), false);
            cancel.cancel();

            Assert::AreEqual<std::string>("123", ss.str());
        }

        TEST_METHOD(LoopSchedule)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            mira::cancellation_source cancel;
            mira::background_dispatcher<32> background;

            mira::state_machine_state<void> one{ "1" }, two{ "2" }, three{ "3" };

            mira::signal signal;

            make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                signal.set();
            });

            std::stringstream ss;

            WorkOn(ss, sched, one, cancel, background);
            WorkOn(ss, sched, two, cancel, background);
            WorkOn(ss, sched, three, cancel, background);

            signal.wait(mira::cancellation::none(), false);
            cancel.cancel();

            Assert::AreEqual<std::string>("123123", ss.str());
        }

        TEST_METHOD(JumpAroundTheGraph)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };
            mira::cancellation_source cancel;
            mira::background_dispatcher<32> background;

            mira::state_machine_state<void> one{ "1" }, two{ "2" }, three{ "3" };

            mira::signal signal;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(two, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(three, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                signal.set();
            });

            std::stringstream ss;

            WorkOn(ss, sched, one, cancel, background);
            WorkOn(ss, sched, two, cancel, background);
            WorkOn(ss, sched, three, cancel, background);

            signal.wait(mira::cancellation::none(), false);
            cancel.cancel();

            Assert::AreEqual<std::string>("1311213", ss.str());
        }

        TEST_METHOD(CancelInOnMethodDoesntBlockSchedule)
        {
            mira::state_machine_driver driver{};
            mira::state_machine_observer sched{ driver };

            mira::state_machine_state<void> one{ "One" }, two{ "Two" };
            mira::cancellation_source cancel;

            bool ranContinuation = false;

            mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                return driver.move_to(one, mira::cancellation::none());
            }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
            {
                ranContinuation = true;
            });

            make_task(mira::inline_scheduler, cancel, [&]
            {
                return sched.on(one, mira::inline_scheduler, cancel, [&]
                {
                    cancel.cancel();
                });
            });

            Assert::IsTrue(ranContinuation);
        }

        struct Runtime
        {
            mira::state_machine_driver Driver{};
            mira::state_machine_observer Scheduler{ Driver };
            mira::manual_dispatcher<32> Dispatcher;
            Mediator Mediator{ Dispatcher };

            Context Context{ Mediator, Scheduler, Dispatcher };

            std::unique_ptr<InitializationWorker> m_init;
            std::unique_ptr<TrackingWorker> m_tracking;

            mira::task<void> Start()
            {
                return InitSchedule();
            }

            bool FailedOnce = false;
            bool Completed = false;

            mira::task<void> Die()
            {
                return mira::task_from_error<void>(std::errc::owner_dead);
            }

            mira::task<void> TrackingSchedule()
            {
                if (!m_tracking)
                {
                    m_tracking = std::make_unique<TrackingWorker>(Context);
                }

                return mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    // do a tracking read
                    return Driver.move_to(TrackingRead, mira::cancellation::none());
                }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    // then a tracking write
                    return Driver.move_to(TrackingWrite, mira::cancellation::none());
                }).then(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    if (!FailedOnce && m_tracking->Iterations >= 2)
                    {
                        return m_tracking->ShutdownAsync().
                            then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>&)
                            {
                                FailedOnce = true;
                                m_tracking = nullptr;

                                return InitSchedule();
                            });
                    }
                    else if (FailedOnce && m_tracking->Iterations >= 2)
                    {
                        return m_tracking->ShutdownAsync().
                            then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>&)
                        {
                            m_tracking.reset();

                            return Die();
                        });
                    }
                    else
                    {
                        return TrackingSchedule();
                    }
                });
            }

            mira::task<void> InitSchedule()
            {
                if (!m_init)
                {
                    m_init = std::make_unique<InitializationWorker>(Context);
                }

                return mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [&]
                {
                    return Driver.move_to(TrackingInit, mira::cancellation::none());
                }).then(mira::inline_scheduler, mira::cancellation::none(), [&](const bool& initialized)
                {
                    if (!initialized)
                    {
                        return InitSchedule();
                    }
                    else
                    {
                        return m_init->ShutdownAsync()
                            .then(mira::inline_scheduler, mira::cancellation::none(), [&]
                            {
                                m_init.reset();

                                return TrackingSchedule();
                            });
                    }
                });
            }
        };

        TEST_METHOD(DynamicRuntime)
        {
            Runtime runtime;

            bool completed = false;

            runtime.Start().then(mira::inline_scheduler, mira::cancellation::none(), [&](const mira::expected<void>& result)
            {
                Assert::IsTrue(result.error() == std::errc::owner_dead);
                completed = true;
            });

            do
            {
                if (!completed)
                {
                    runtime.Mediator.send<ImageReceived>({});
                }
            } while (runtime.Dispatcher.tick(mira::cancellation::none()));
        }
    };
}
