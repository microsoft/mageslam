#include <CppUnitTest.h>

#include "arcana\containers\sorted_vector.h"
#include "arcana\containers\unique_vector.h"
#include "arcana\containers\ticketed_collection.h"

#include <numeric>
#include <algorithm>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(ContainerUnitTest)
    {
        TEST_METHOD(SortedVectorInsert)
        {
            mira::sorted_vector<int> elements{ 2, 3, 1, 4 };

            std::vector<int> desired{ 1, 2, 3, 4 };
            Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");

            desired = { 1, 2, 3, 4, 5 };
            elements.insert(5);
            Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");
        }

        TEST_METHOD(UniqueVectorInsert)
        {
            mira::unique_vector<int> elements{ 2, 3, 1, 4 };

            std::vector<int> desired = { 1, 2, 3, 4 };
            elements.insert(3);
            Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");

            desired = { 1, 2, 3, 4 };
            elements.insert(desired.begin(), desired.end());
            Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");
        }

        TEST_METHOD(SortedVectorMerge)
        {
            {
                mira::sorted_vector<int> elements{ 2, 3, 1, 4 };

                std::vector<int> desired{ 1, 1, 2, 2, 3, 3, 4, 4 };
                elements.merge(elements);
                Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");
            }
            {
                mira::unique_vector<int> elements{ 1, 2, 3, 4 };

                std::vector<int> desired = { 1, 2, 3, 4 };
                elements.merge(elements);
                Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");
            }
            {
                mira::sorted_vector<int> elements{ 2, 3, 1, 4 };

                mira::sorted_vector<int> empty;
                std::vector<int> desired = { 1, 2, 3, 4 };
                elements.merge(empty);
                Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");
            }
            {
                mira::unique_vector<int> elements{ 2, 3, 1, 4 };

                mira::unique_vector<int> empty;
                std::vector<int> desired = { 1, 2, 3, 4 };
                elements.merge(empty);
                Assert::IsTrue(equal(elements.begin(), elements.end(), desired.begin(), desired.end()), L"elements should be sorted");
            }
        }

        static auto insert_item(mira::ticketed_collection<int>& items, int i, std::mutex& mutex)
        {
            std::lock_guard<std::mutex> guard{ mutex };
            return items.insert(i, mutex);
        }

        TEST_METHOD(TicketedCollectionManipulation)
        {
            mira::ticketed_collection<int> items;
            std::mutex mutex;

            for (int i = 0; i < 10; ++i)
            {
                auto el = insert_item(items, i, mutex);
            }

            Assert::AreEqual<size_t>(0, items.size());
            Assert::IsTrue(items.empty());

            {
                auto elHeld = insert_item(items, 10, mutex);

                Assert::AreEqual<size_t>(1, items.size());
                Assert::IsFalse(items.empty());

                int count = 0;
                for (auto& el : items)
                {
                    count++;
                    Assert::AreEqual(10, el);
                }
                Assert::AreEqual(1, count);
            }

            Assert::AreEqual<size_t>(0, items.size());
            Assert::IsTrue(items.empty());
        }
    };
}
