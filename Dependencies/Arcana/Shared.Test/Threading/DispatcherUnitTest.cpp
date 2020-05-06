#include <CppUnitTest.h>

#include <arcana\threading\dispatcher.h>

#include <numeric>
#include <algorithm>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(DispatcherUnitTest)
    {
        TEST_METHOD(DispatcherLeakCheck)
        {
            std::weak_ptr<int> weak;

            {
                mira::background_dispatcher<32> dis;

                std::shared_ptr<int> strong = std::make_shared<int>(10);
                weak = strong;

                std::promise<void> signal;

                dis.queue([strong, &signal]{
                    signal.set_value();
                });

                signal.get_future().get();
            }

            Assert::IsTrue(weak.expired());
        }
    };
}
