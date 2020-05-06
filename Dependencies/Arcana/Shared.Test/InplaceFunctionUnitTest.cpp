#include <CppUnitTest.h>

#include <arcana\functional\inplace_function.h>

#include <memory>

using namespace std;

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(InplaceFunctionUnitTest)
    {
        TEST_METHOD(MoveSemanticsInvalidatesMovedFunction)
        {
            stdext::inplace_function<void()> source = [] {};
            stdext::inplace_function<void()> dest = std::move(source);
            Assert::IsTrue(!source, L"Once the function is moved, I shouldn't be able to call it with a bunch of invalid data");
        }

        TEST_METHOD(MovedFunctionGetsPropertyDestroyed)
        {
            std::weak_ptr<int> weak;

            {
                const std::shared_ptr<int> value = std::make_shared<int>(10);
                weak = value;

                stdext::inplace_function<void()> source = [value] {};
                stdext::inplace_function<void()> dest = std::move(source);
            }

            Assert::IsTrue(weak.expired());
        }
    };
}
