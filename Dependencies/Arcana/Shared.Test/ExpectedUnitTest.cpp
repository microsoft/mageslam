#include <CppUnitTest.h>

#include <arcana\expected.h>
#include <arcana\errors.h>

#include <memory>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(ExpectedUnitTest)
    {
        TEST_METHOD(ExpectedCopy)
        {
            auto data = std::make_shared<int>(42);
            std::weak_ptr<int> weak = data;

            {
                mira::expected<std::shared_ptr<int>> exp{ data };

                mira::expected<std::shared_ptr<int>> other{ exp };

                exp = other;
            }

            data.reset();
            Assert::IsTrue(weak.lock() == nullptr);
        }

        TEST_METHOD(ExpectedMove)
        {
            auto data = std::make_shared<int>(42);
            std::weak_ptr<int> weak = data;

            {
                mira::expected<std::shared_ptr<int>> exp{ std::move(data) };

                mira::expected<std::shared_ptr<int>> other{ std::move(exp) };

                exp = std::move(other);
            }

            Assert::IsTrue(weak.lock() == nullptr);
        }

        TEST_METHOD(ExpectedCopyError)
        {
            auto data = std::make_shared<int>(42);
            std::weak_ptr<int> weak = data;

            {
                mira::expected<std::shared_ptr<int>> exp{ std::move(data) };

                mira::expected<std::shared_ptr<int>> other{ std::errc::operation_canceled };

                exp = other;
            }

            Assert::IsTrue(weak.lock() == nullptr);
        }

        TEST_METHOD(ExpectedMoveError)
        {
            auto data = std::make_shared<int>(42);
            std::weak_ptr<int> weak = data;

            {
                mira::expected<std::shared_ptr<int>> exp{ std::move(data) };

                mira::expected<std::shared_ptr<int>> other{ std::errc::operation_canceled };

                exp = std::move(other);
            }

            Assert::IsTrue(weak.lock() == nullptr);
        }

        TEST_METHOD(ExpectedAssignError)
        {
            auto data = std::make_shared<int>(42);
            std::weak_ptr<int> weak = data;

            {
                mira::expected<std::shared_ptr<int>> exp{ std::move(data) };

                exp = std::errc::operation_canceled;
            }

            Assert::IsTrue(weak.lock() == nullptr);
        }
    };
}
