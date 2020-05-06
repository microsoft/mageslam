#include <CppUnitTest.h>

#include <arcana\functional\inplace_function.h>
#include <arcana\messaging\mediator.h>
#include <arcana\expected.h>

#include <fstream>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(MediatorUnitTest)
    {
        struct tracer
        {
            tracer(std::ofstream& stream)
                : m_stream{ &stream }
            {}

            tracer()
                : m_stream{ nullptr }
            {}

            tracer(const tracer& other)
                : m_stream{ other.m_stream }
            {
                *m_stream << "T(const T&)" << std::endl;
            }

            tracer(tracer&& other)
                : m_stream{ other.m_stream }
            {
                *m_stream << "T(T&&)" << std::endl;
                other.m_stream = nullptr;
            }

            tracer& operator =(const tracer& other)
            {
                *m_stream << "T& operator=(const T&)" << std::endl;
                m_stream = other.m_stream;
                return *this;
            }

            tracer& operator =(tracer&& other)
            {
                *m_stream << "T& operator=(T&&)" << std::endl;
                m_stream = other.m_stream;
                other.m_stream = nullptr;
                return *this;
            }

            std::ofstream* m_stream;
        };

        static void call(std::shared_ptr<int> ptr, std::unique_ptr<int> unique)
        {
            std::ofstream blah{ "output.txt" };
            blah << "shared: " << *ptr << " unique: " << *unique << std::endl;
        }

        static void trace_value(tracer t)
        {
            *t.m_stream << "done" << std::endl;
        }

        static void trace_ref(tracer& t)
        {
            *t.m_stream << "done" << std::endl;
        }

        static void trace_mov(tracer&& t)
        {
            *t.m_stream << "done" << std::endl;
        }

        TEST_METHOD(InplaceFunctionForwardingSemantics)
        {
            std::ofstream file{ "tracing.txt", std::ios::trunc };

            {
                file << "trace_value1" << std::endl;
                stdext::inplace_function<void(tracer)> func{ &trace_value };
                func(tracer{ file });
            }

            {
                file << "trace_value2" << std::endl;
                stdext::inplace_function<void(tracer)> func{ &trace_value };
                tracer t{ file };
                func(t);
            }

            {
                file << "trace_value3" << std::endl;
                stdext::inplace_function<void(tracer)> func{ &trace_value };
                tracer t{ file };
                func(std::move(t));
            }

            {
                file << "trace_ref" << std::endl;
                stdext::inplace_function<void(tracer&)> func{ &trace_ref };
                tracer t{ file };
                func(t);
            }

            {
                file << "trace_mov" << std::endl;
                stdext::inplace_function<void(tracer&&)> func{ &trace_mov };
                func(tracer{ file });
            }

            {
                stdext::inplace_function<void(std::shared_ptr<int>, std::unique_ptr<int>)> func{ &call };
                func(std::make_shared<int>(0), std::make_unique<int>(1));
            }
        }

        struct one
        {
            int value{};
        };

        struct two
        {
            std::string message{};
        };

        struct three
        {
            int mat[3]{};
        };

        TEST_METHOD(RouterSingleEvent)
        {
            mira::router<one> rout;

            int received = 10;
            auto reg1 = rout.add_listener<one>([&received](const one& evt)
            {
                received = evt.value;
            });

            rout.fire(one{ 1 });

            Assert::AreEqual(1, received);
        }

        TEST_METHOD(RouterNonPODType)
        {
            mira::router<mira::expected<std::shared_ptr<int>>> rout;

            auto lambda = std::make_shared<int>(0);
            std::weak_ptr<int> wlambda = lambda;
            std::weak_ptr<int> wshared;

            int received = 0;

            {
                auto reg1 = rout.add_listener<mira::expected<std::shared_ptr<int>>>([&received, lambda = std::move(lambda)](const mira::expected<std::shared_ptr<int>>& evt)
                {
                    received = *evt.value();
                });

                auto shared = std::make_shared<int>(10);
                wshared = shared;

                rout.fire(mira::expected<std::shared_ptr<int>>{ std::move(shared) });
            }

            Assert::AreEqual(10, received);
            Assert::IsTrue(wshared.expired());
            Assert::IsTrue(wlambda.expired());
        }

        TEST_METHOD(RouterMultipleEvent)
        {
            mira::router<one, two> rout;

            int received = 10;
            auto reg1 = rout.add_listener<one>([&received](const one& /*evt*/)
            {
                received = 1;
            });

            auto reg2 = rout.add_listener<two>([&received](const two& /*evt*/)
            {
                received = 2;
            });

            rout.fire(one{});

            Assert::AreEqual(1, received);

            rout.fire(one{});

            Assert::AreEqual(1, received);

            rout.fire(two{});

            Assert::AreEqual(2, received);

            rout.fire(one{});

            Assert::AreEqual(1, received);
        }

        TEST_METHOD(RouterUnregister)
        {
            mira::router<one, two> rout;

            int received = 0;
            {
                auto reg = rout.add_listener<one>([&](const one& /*evt*/)
                {
                    received++;
                });

                rout.fire(one{});
            }
            rout.fire(one{});
            Assert::AreEqual(1, received);

            {
                auto reg = rout.add_listener<one>([&](const one& /*evt*/)
                {
                    received++;
                });

                rout.fire(one{});
            }
            Assert::AreEqual(2, received);

            rout.fire(one{});
            Assert::AreEqual(2, received);

            std::unique_ptr<mira::ticket> ticket;
            ticket = std::make_unique<mira::ticket>(rout.add_listener<one>([&](const one& /*evt*/)
            {
                received++;

                ticket.reset();
            }));
            rout.fire(one{});
            Assert::AreEqual(3, received);
            rout.fire(one{});
            Assert::AreEqual(3, received);

            std::unique_ptr<mira::ticket> ticket2;
            ticket2 = std::make_unique<mira::ticket>(rout.add_listener<one>([&](const one& /*evt*/)
            {
                received++;

                ticket2.reset();

                rout.fire(one{});
            }));

            std::unique_ptr<mira::ticket> ticket3;
            ticket3 = std::make_unique<mira::ticket>(rout.add_listener<one>([&](const one& /*evt*/)
            {
                received *= 2;

                ticket3.reset();

                rout.fire(one{});
            }));

            rout.fire(one{});

            Assert::AreEqual(8, received);
        }

        TEST_METHOD(RouterRegisterOther)
        {
            mira::router<one, two> rout;

            int received = 0;
            std::unique_ptr<mira::ticket> twol;
            auto reg = rout.add_listener<one>([&](const one& /*evt*/)
            {
                received++;

                twol = std::make_unique<mira::ticket>(rout.add_listener<two>([&](const two& /*other*/)
                {
                    received *= 2;
                }));

                rout.fire(two{});
            });

            rout.fire(one{});
            Assert::AreEqual(2, received);

            rout.fire(two{});
            Assert::AreEqual(4, received);
        }

        TEST_METHOD(RouterRegisterSame)
        {
            mira::router<one, two> rout;

            int received = 0;
            std::unique_ptr<mira::ticket> twol, onel;
            auto reg = std::make_unique<mira::ticket>(rout.add_listener<one>([&](const one& /*evt*/)
            {
                received++;

                twol = std::make_unique<mira::ticket>(rout.add_listener<two>([&](const two& /*other*/)
                {
                    received *= 2;
                }));

                rout.fire(two{});

                onel = std::make_unique<mira::ticket>(rout.add_listener<one>([&](const one& /*otherone*/)
                {
                    received += 7;
                }));
            }));

            rout.fire(one{});
            Assert::AreEqual(2, received);

            reg.reset();

            rout.fire(one{});
            Assert::AreEqual(9, received);
        }

        TEST_METHOD(RouterFire)
        {
            mira::router<one, two> rout;

            int received = 0;
            auto oreg = rout.add_listener<one>([&](const one& /*evt*/)
            {
                received++;

                rout.fire(two{});
            });

            {
                auto treg = rout.add_listener<two>([&](const two& /*evt*/)
                {
                    received *= 2;
                });

                rout.fire(one{});

                Assert::AreEqual(2, received);

                rout.fire(two{});
                Assert::AreEqual(4, received);
            }


            rout.fire(one{});

            Assert::AreEqual(5, received);

            std::unique_ptr<mira::ticket> treg;
            treg = std::make_unique<mira::ticket>(rout.add_listener<two>([&](const two& /*evt*/)
            {
                received *= 2;

                treg.reset();

                rout.fire(one{});
            }));

            rout.fire(one{});

            Assert::AreEqual(13, received);
        }

        TEST_METHOD(DispatcherOrdering)
        {
            mira::manual_dispatcher<32> dis;
            dis.set_affinity({ std::this_thread::get_id() });

            int value = -1;
            dis.queue([&]
            {
                value = 1;
            });

            dis.queue([&]
            {
                value *= 2;
            });

            dis.queue([&]
            {
                value -= 5;
            });

            Assert::AreEqual(-1, value);

            mira::cancellation_source source;
            dis.tick(source);

            Assert::AreEqual(-3, value);
        }

        TEST_METHOD(DispatcherOrderingRecursive)
        {
            mira::manual_dispatcher<32> dis;
            dis.set_affinity({ std::this_thread::get_id() });

            int value = -1;
            dis.queue([&]
            {
                value = 1;

                dis.queue([&]
                {
                    value *= 5;
                });
            });

            dis.queue([&]
            {
                value *= 2;

                dis.queue([&]
                {
                    value -= 3;
                });
            });

            dis.queue([&]
            {
                value -= 5;

                dis.queue([&]
                {
                    value *= -2;
                });
            });

            Assert::AreEqual(-1, value);

            mira::cancellation_source source;
            dis.tick(source);

            Assert::AreEqual(-3, value);

            dis.tick(source);

            Assert::AreEqual(36, value);
        }

        TEST_METHOD(SingleEvent)
        {
            mira::manual_dispatcher<32> dis;
            mira::mediator<decltype(dis), one> med{ dis };

            int received = 10;
            auto reg = med.add_listener<one>([&received](const one& evt)
            {
                received = evt.value;
            });

            med.send(one{ 1 });

            mira::cancellation_source source;
            dis.tick(source);

            Assert::AreEqual(1, received);
        }

        TEST_METHOD(MediatorNonPODType)
        {
            mira::manual_dispatcher<32> dis;
            mira::mediator<decltype(dis), mira::expected<std::shared_ptr<int>>> rout{ dis };

            auto lambda = std::make_shared<int>(0);
            std::weak_ptr<int> wlambda = lambda;
            std::weak_ptr<int> wshared;

            int received = 0;

            {
                auto reg1 = rout.add_listener<mira::expected<std::shared_ptr<int>>>([&received, lambda = std::move(lambda)](const mira::expected<std::shared_ptr<int>>& evt)
                {
                    received = *evt.value();
                });

                auto shared = std::make_shared<int>(10);
                wshared = shared;

                rout.send(mira::expected<std::shared_ptr<int>>{ std::move(shared) });

                mira::cancellation_source source;
                dis.tick(source);
            }

            Assert::AreEqual(10, received);
            Assert::IsTrue(wshared.expired());
            Assert::IsTrue(wlambda.expired());
        }

        TEST_METHOD(MultipleEvents)
        {
            mira::manual_dispatcher<64> dis; // Manual dispatcher size is selected to be big enough to fit pointer to mediator and maximal size out of "one", "two" and "three" below.
            mira::mediator<decltype(dis), one, two, three> med{ dis };
            mira::ticket_scope registrations;

            int received = 10;
            registrations += med.add_listener<one>([&received](const one& evt)
            {
                received = evt.value;
            });

            registrations += med.add_listener<two>([&received](const two& evt)
            {
                for (char c : evt.message)
                {
                    received *= c;
                }
            });

            registrations += med.add_listener<three>([&received](const three& evt)
            {
                for (int c : evt.mat)
                {
                    received += c;
                }
            });

            med.send(one{ 3 });
            med.send(two{ "two" });
            med.send(three{ {1, 2, 3} });

            mira::cancellation_source source;
            dis.tick(source);

            Assert::AreEqual(4596738, received);
        }
    };
}
