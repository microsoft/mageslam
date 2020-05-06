#include <CppUnitTest.h>

#include <arcana\type_traits.h>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace
{
    enum TestCStyleEnum
    {
        Value1 = 1,
        Value2 = 3,
        Value3 = 157
    };

    enum class TestEnumClass : long long
    {
        Value1 = 2,
        Value2 = -1,
        Value3 = 256
    };
}

namespace UnitTests
{

    TEST_CLASS(TypeTraitsTest)
    {

        TEST_METHOD(UnderlyingCast_WithCStyleEnum_ReturnsCorrectType)
        {
            constexpr auto underlyingValue1 = mira::underlying_cast(TestCStyleEnum::Value1);
            static_assert(std::is_same<std::decay_t<decltype(underlyingValue1)>, std::underlying_type_t<TestCStyleEnum>>::value, "");
        }

        TEST_METHOD(UnderlyingCast_WithCStyleEnum_ReturnsCorrectValues)
        {
            constexpr auto underlyingValue1 = mira::underlying_cast(TestCStyleEnum::Value1);
            constexpr auto underlyingValue2 = mira::underlying_cast(TestCStyleEnum::Value2);
            constexpr auto underlyingValue3 = mira::underlying_cast(TestCStyleEnum::Value3);

            static_assert(underlyingValue1 == 1, "");
            static_assert(underlyingValue2 == 3, "");
            static_assert(underlyingValue3 == 157, "");
        }

        TEST_METHOD(UnderlyingCast_WithEnumClass_ReturnsCorrectType)
        {
            constexpr auto underlyingValue1 = mira::underlying_cast(TestEnumClass::Value1);

            static_assert(std::is_same<std::decay_t<decltype(underlyingValue1)>, std::underlying_type_t<TestEnumClass>>::value, "");
        }

        TEST_METHOD(UnderlyingCast_WithEnumClass_ReturnsCorrectValues)
        {
            constexpr auto underlyingValue1 = mira::underlying_cast(TestEnumClass::Value1);
            constexpr auto underlyingValue2 = mira::underlying_cast(TestEnumClass::Value2);
            constexpr auto underlyingValue3 = mira::underlying_cast(TestEnumClass::Value3);

            static_assert(underlyingValue1 == 2, "");
            static_assert(underlyingValue2 == -1, "");
            static_assert(underlyingValue3 == 256, "");
        }

        TEST_METHOD(InvokeOptionalParameter_InvokesTheRightFunction)
        {
            bool invokedvoid = false;
            auto funcvoid = [&] { invokedvoid = true; };

            bool invokedp = false;
            auto funcp = [&](int value)
            {
                Assert::AreEqual(10, value);
                invokedp = true;
            };

            mira::invoke_with_optional_parameter(funcvoid, 10);
            Assert::IsTrue(invokedvoid);

            mira::invoke_with_optional_parameter(funcp, 10);
            Assert::IsTrue(invokedp);
        }

        TEST_METHOD(CountTrueConditionalExpressions)
        {
            Assert::AreEqual<size_t>(3, mira::count_true<std::true_type, std::true_type, std::true_type>::value);
            Assert::AreEqual<size_t>(0, mira::count_true<>::value);
            Assert::AreEqual<size_t>(1, mira::count_true<std::true_type>::value);
            Assert::AreEqual<size_t>(0, mira::count_true<std::false_type>::value);
            Assert::AreEqual<size_t>(2, mira::count_true<std::true_type, std::false_type, std::true_type>::value);
        }

        TEST_METHOD(FindFirstTrueConditionalExpressionIndex)
        {
            Assert::AreEqual<size_t>(0, mira::find_first_index<std::true_type, std::true_type, std::true_type>::value);
            Assert::AreEqual<size_t>(1, mira::find_first_index<std::false_type, std::true_type, std::true_type>::value);
            Assert::AreEqual<size_t>(1, mira::find_first_index<std::false_type, std::true_type>::value);
            Assert::AreEqual<size_t>(0, mira::find_first_index<std::true_type, std::false_type>::value);
            Assert::AreEqual<size_t>(2, mira::find_first_index<std::false_type, std::false_type, std::true_type>::value);

            // test the end condition
            Assert::AreEqual<size_t>(0, mira::find_first_index<>::value);
            Assert::AreEqual<size_t>(3, mira::find_first_index<std::false_type, std::false_type, std::false_type>::value);
        }
    };
}
