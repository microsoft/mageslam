#include <CppUnitTest.h>

#include <arcana/propertybag.h>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(PropertyBagUnitTest)
    {
        PROPERTYBAG(ConsoleSettings,
            PROPERTY(float, FloatProperty, 1.0f)
            PROPERTY(int, IntProperty, 10)
        );

        PROPERTYBAG(GroupOne,
            BAG_PROPERTY(ConsoleSettings)
        );

        TEST_METHOD(PropertyBag_WhenCopied_ValuesSameInTarget)
        {
            ConsoleSettings first{};
            first.FloatProperty = 2.0f;
            first.IntProperty= 5;

            ConsoleSettings second = first;

            ConsoleSettings third{ second };
            third.FloatProperty = 10.f;

            second = third;

            Assert::AreEqual(2.f, first.FloatProperty.value);
            Assert::AreEqual(5, first.IntProperty.value);

            Assert::AreEqual(10.f, second.FloatProperty.value);
            Assert::AreEqual(5, second.IntProperty.value);

            Assert::AreEqual(10.f, third.FloatProperty.value);
            Assert::AreEqual(5, third.IntProperty.value);
        }
    };
}
