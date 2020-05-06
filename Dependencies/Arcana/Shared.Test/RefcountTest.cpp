#include <CppUnitTest.h>

#include <arcana\reference_counter.h>

#include <deque>
#include <vector>
#include <future>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    namespace
    {
        void CountUp(mira::reference_counter &counter, uint32_t max)
        {
            for (uint32_t i = 0; i < max; ++i)
            {
                counter.add_reference();
            }
        }
        void CountDown(mira::reference_counter &counter, uint32_t max)
        {
            for (uint32_t i = 0; i < max; ++i)
            {
                counter.remove_reference();
            }
        }
    }

    TEST_CLASS(RefcountTest)
    {
        TEST_METHOD(SimpleRefcount)
        {
            mira::reference_counter counter;
            const uint32_t max = 10;
            uint32_t i = 0;
            while (i < max)
            {
                counter.add_reference();
                Assert::AreEqual(++i, counter.current_count());
                Assert::AreEqual(1.0f, counter.percent_of_max());
            }
            while (i > 0)
            {
                counter.remove_reference();
                Assert::AreEqual(--i, counter.current_count());
                Assert::AreEqual(static_cast<float>(i)/max, counter.percent_of_max());
            }
        }

        TEST_METHOD(SimpleScopedRefcount)
        {
            std::deque<mira::scoped_reference> refs;
            mira::reference_counter counter;
            const uint32_t max = 10;
            uint32_t i = 0;
            while (i < max)
            {
                refs.push_back(mira::scoped_reference(counter));
                Assert::AreEqual(++i, counter.current_count());
                Assert::AreEqual(1.0f, counter.percent_of_max());
            }
            while (i > 0)
            {
                refs.pop_front();
                Assert::AreEqual(--i, counter.current_count());
                Assert::AreEqual(static_cast<float>(i) / max, counter.percent_of_max());
            }
        }

        TEST_METHOD(ThreadedRefcount)
        {
            const uint32_t max = 10;
            const uint32_t numThreads = 10;
            std::vector<std::future<void>> threads;

            // Count up in many threads then down in many threads
            {
                mira::reference_counter counter;

                for (uint32_t i = 0; i < numThreads; i++)
                {
                    threads.push_back(std::async(std::launch::async, CountUp, std::ref(counter), max));
                }
                threads.clear();

                Assert::AreEqual(max * numThreads, counter.current_count());
                Assert::AreEqual(1.0f, counter.percent_of_max());

                for (uint32_t i = 0; i < numThreads; i++)
                {
                    threads.push_back(std::async(std::launch::async, CountDown, std::ref(counter), max));
                }
                threads.clear();

                // Explicit casting required due to different definitions of uint32_t between MSVC and QDSP
                Assert::AreEqual<uint32_t>(0u, counter.current_count());
                Assert::AreEqual(0.0f / max, counter.percent_of_max());
            }

            // Count up and down in many threads at once
            {
                mira::reference_counter counter;


                for (uint32_t i = 0; i < numThreads; i++)
                {
                    threads.push_back(std::async(std::launch::async, CountUp, std::ref(counter), max));
                    threads.push_back(std::async(std::launch::async, CountDown, std::ref(counter), max));
                }
                threads.clear();

                Assert::AreEqual<uint32_t>(0u, counter.current_count());
                Assert::AreEqual(0.0f / max, counter.percent_of_max());
            }
        }

        TEST_METHOD(RValueScopedRefcount)
        {
            mira::reference_counter counter1;
            mira::reference_counter counter2;
            Assert::AreEqual<uint32_t>(0u, counter1.current_count());
            Assert::AreEqual<uint32_t>(0u, counter2.current_count());

            {
                // Constructor
                mira::scoped_reference r1(counter1);
                mira::scoped_reference r2(counter2);
                Assert::AreEqual<uint32_t>(1u, counter1.current_count());
                Assert::AreEqual<uint32_t>(1u, counter2.current_count());

                {
                    // Copy constructor
                    mira::scoped_reference r3(r1);
                    mira::scoped_reference r4(r2);
                    Assert::AreEqual<uint32_t>(2u, counter1.current_count());
                    Assert::AreEqual<uint32_t>(2u, counter2.current_count());
                }
                Assert::AreEqual<uint32_t>(1u, counter1.current_count());
                Assert::AreEqual<uint32_t>(1u, counter2.current_count());

                {
                    // Move constructor
                    mira::scoped_reference r3(std::move(r1));
                    mira::scoped_reference r4(std::move(r2));
                    Assert::AreEqual<uint32_t>(1u, counter1.current_count());
                    Assert::AreEqual<uint32_t>(1u, counter2.current_count());

                    // Move assignment
                    r1 = mira::scoped_reference(counter1);
                    r2 = mira::scoped_reference(counter2);
                    Assert::AreEqual<uint32_t>(2u, counter1.current_count());
                    Assert::AreEqual<uint32_t>(2u, counter2.current_count());
                }
                Assert::AreEqual<uint32_t>(1u, counter1.current_count());
                Assert::AreEqual<uint32_t>(1u, counter2.current_count());
                Assert::AreEqual(0.5f, counter1.percent_of_max());
                Assert::AreEqual(0.5f, counter2.percent_of_max());

                // Assignment assignment
                r1 = r2;
                Assert::AreEqual<uint32_t>(0u, counter1.current_count());
                Assert::AreEqual<uint32_t>(2u, counter2.current_count());
                r2 = r1;
                Assert::AreEqual<uint32_t>(0u, counter1.current_count());
                Assert::AreEqual<uint32_t>(2u, counter2.current_count());

                std::function <mira::scoped_reference(mira::reference_counter &counter)>f = [](mira::reference_counter &counter)
                {
                    return mira::scoped_reference(counter);
                };
                mira::scoped_reference r3 = f(counter1);
                Assert::AreEqual<uint32_t>(1u, counter1.current_count());
                Assert::AreEqual<uint32_t>(2u, counter2.current_count());

                r1 = r2 = r3;
                Assert::AreEqual<uint32_t>(3u, counter1.current_count());
                Assert::AreEqual<uint32_t>(0u, counter2.current_count());

                r3 = r2 = r1 = mira::scoped_reference(counter2);
                Assert::AreEqual<uint32_t>(0u, counter1.current_count());
                Assert::AreEqual<uint32_t>(3u, counter2.current_count());
            }
            Assert::AreEqual<uint32_t>(0u, counter1.current_count());
            Assert::AreEqual<uint32_t>(0u, counter2.current_count());
            Assert::AreEqual(0.0f, counter1.percent_of_max());
            Assert::AreEqual(0.0f, counter2.percent_of_max());
        }
    };
}
