#include <CppUnitTest.h>

#include <arcana\either.h>

#include <memory>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(EitherUnitTest)
    {
        using either_number = mira::either<std::shared_ptr<int>, std::shared_ptr<float>>;

        TEST_METHOD(EitherCopy)
        {
            auto data1 = std::make_shared<int>(42);
            auto data2 = std::make_shared<float>(42.0f);

            std::weak_ptr<int> weak1 = data1;
            std::weak_ptr<float> weak2 = data2;

            {
                either_number exp{ data1 };

                Assert::IsTrue(exp.has_first());
                Assert::IsTrue(!exp.has_second());
                Assert::AreEqual(42, *exp.first());

                exp = data2;

                Assert::IsTrue(!exp.has_first());
                Assert::IsTrue(exp.has_second());
                Assert::AreEqual(42.0f, *exp.second());

                either_number other{ exp };

                Assert::IsTrue(!other.has_first());
                Assert::IsTrue(other.has_second());
                Assert::AreEqual(42.0f, *other.second());

                other = data1;

                Assert::IsTrue(other.has_first());
                Assert::IsTrue(!other.has_second());
                Assert::AreEqual(42, *other.first());

                exp = other;

                Assert::IsTrue(exp.has_first());
                Assert::IsTrue(!exp.has_second());
                Assert::AreEqual(42, *exp.first());
            }

            data1.reset();
            data2.reset();

            Assert::IsTrue(weak1.lock() == nullptr);
            Assert::IsTrue(weak2.lock() == nullptr);
        }

        TEST_METHOD(EitherMove)
        {
            auto data1 = std::make_shared<int>(42);
            auto data2 = std::make_shared<float>(42.0f);

            std::weak_ptr<int> weak1 = data1;
            std::weak_ptr<float> weak2 = data2;

            {
                auto otherdata1 = data1;
                either_number exp{ std::move(data1) };

                Assert::IsTrue(exp.has_first());
                Assert::IsTrue(!exp.has_second());
                Assert::AreEqual(42, *exp.first());

                exp = std::move(data2);

                Assert::IsTrue(!exp.has_first());
                Assert::IsTrue(exp.has_second());
                Assert::AreEqual(42.0f, *exp.second());

                either_number other{ std::move(exp) };

                Assert::IsTrue(!other.has_first());
                Assert::IsTrue(other.has_second());
                Assert::AreEqual(42.0f, *other.second());

                other = std::move(otherdata1);

                Assert::IsTrue(other.has_first());
                Assert::IsTrue(!other.has_second());
                Assert::AreEqual(42, *other.first());

                exp = std::move(other);

                Assert::IsTrue(exp.has_first());
                Assert::IsTrue(!exp.has_second());
                Assert::AreEqual(42, *exp.first());
            }

            data1.reset();
            data2.reset();

            Assert::IsTrue(weak1.lock() == nullptr);
            Assert::IsTrue(weak2.lock() == nullptr);
        }
    };
}
