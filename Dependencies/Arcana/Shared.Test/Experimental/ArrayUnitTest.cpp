//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include <CppUnitTest.h>

#include <arcana/experimental/array.h>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(ArrayUnitTest)
    {
        TEST_METHOD(GivenArray_WhenConstructedWithMakeArray_VerifyElementsAreInArray)
        {
            // Auto-deduced type should be the same as the explicitly defined type.
            auto arr1 = mira::make_array(1, 2, 3);
            std::array<int, 3> arr2{ 1, 2, 3 };

            static_assert(std::is_same<decltype(arr1), decltype(arr2)>::value, "Differing array types");
            static_assert(arr1.size() == arr2.size(), "Differing array lengths");

            Assert::IsTrue(std::equal(arr1.begin(), arr1.end(), arr2.begin(), arr2.end()), L"Different elements");
        }
    };
}
