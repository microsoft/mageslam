#include <CppUnitTest.h>

#include <arcana/expected.h>

#ifndef XRAY_ENABLED
    #define XRAY_ENABLED
#endif

#include <arcana/analysis/xray.h>

#include <numeric>
#include <algorithm>

#include <arcana/analysis/binary_iterator.h>
#include <arcana/analysis/introspector.h>
#include <arcana/analysis/object_trace.h>

#include <type_traits>

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace Microsoft {
    namespace VisualStudio {
        namespace CppUnitTestFramework {
            std::wstring ToString(const long long& t)
            {
                return std::to_wstring(t);
            }
        }
    }
}

namespace UnitTests
{
    struct my_iterator
    {
        std::vector<size_t> sizes;

        void operator()(const void* /*data*/, size_t size)
        {
            sizes.push_back(size);
        }
    };

    namespace garbage
    {
        struct data
        {
            int32_t value = 10;
            std::string message = "Hello";
        };

        struct nested_data
        {
            std::shared_ptr<data> value = std::make_shared<data>();
            std::string message = "Hello";
        };

        void binary_iterate(const data& d, mira::binary_iterator<my_iterator>& iterator)
        {
            iterator.iterate(d.value);
            iterator.iterate(d.message);
        }

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const data& d)
        {
            intro(
                cereal::make_nvp("Value", d.value),
                cereal::make_nvp("Message", d.message)
            );
        }

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const nested_data& d)
        {
            intro(
                cereal::make_nvp("Value", d.value),
                cereal::make_nvp("Message", d.message)
            );
        }

        struct nested_vectors
        {
            std::vector<std::shared_ptr<data>> Ptrs{ std::make_shared<data>(), std::make_shared<data>() };
            std::vector<data> Items{ data{}, data{} };
            std::vector<int32_t> Numbers{ 1, 2, 3 };
            std::vector<data> Empty{ };
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const nested_vectors& d)
        {
            intro(
                cereal::make_nvp("Ptrs", d.Ptrs),
                cereal::make_nvp("Items", d.Items),
                cereal::make_nvp("Numbers", d.Numbers),
                cereal::make_nvp("Empty", d.Empty)
            );
        }
    }

    TEST_CLASS(AnalysisUnitTest)
    {
        TEST_METHOD(TestBinaryIterator)
        {
            mira::binary_iterator<my_iterator> itr{ my_iterator{} };

            garbage::data d{};

            itr.iterate(d);

            Assert::AreEqual<size_t>(2, itr.iterator().sizes.size());
            Assert::AreEqual(sizeof(int32_t), itr.iterator().sizes[0]);
            Assert::AreEqual<size_t>(5, itr.iterator().sizes[1]);
        }

        TEST_METHOD(JsonIntrospectionWithData)
        {
            std::stringstream ss;
            {
                garbage::data d{};

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": {
        "Value": 10,
        "Message": "Hello"
    }
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithPointerToData)
        {
            std::stringstream ss;
            {
                auto d = std::make_shared<garbage::data>();

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": {
        "pointer": {
            "Value": 10,
            "Message": "Hello"
        }
    }
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithTopLevelNullPointer)
        {
            std::stringstream ss;
            {
                std::shared_ptr<garbage::data> d;

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": {
        "pointer": null
    }
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithEmptyIntVector)
        {
            std::stringstream ss;
            {
                std::vector<int32_t> d;

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": []
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithEmptyDataVector)
        {
            std::stringstream ss;
            {
                std::vector<garbage::data> d;

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": []
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithIntVector)
        {
            std::stringstream ss;
            {
                std::vector<int32_t> d{ 1, 2, 3 };

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": [
        1,
        2,
        3
    ]
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithDataVector)
        {
            std::stringstream ss;
            {
                std::vector<garbage::data> d{ {}, {} };

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": [
        {
            "Value": 10,
            "Message": "Hello"
        },
        {
            "Value": 10,
            "Message": "Hello"
        }
    ]
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionIntThenSpan)
        {
            std::stringstream ss;
            {
                int32_t value = 10;
                std::vector<int32_t> empty;
                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(value, empty);
            }

            std::string expected =
R"({
    "value0": 10,
    "value1": []
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionIntThenNamedSpan)
        {
            std::stringstream ss;
            {
                int32_t value = 10;
                gsl::span<int32_t> empty;
                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(value, cereal::make_nvp("Empty", empty));
            }

            std::string expected =
R"({
    "value0": 10,
    "Empty": []
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(NestedVectors)
        {
            std::stringstream ss;
            {
                garbage::nested_vectors d{};

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": {
        "Ptrs": [
            {
                "pointer": {
                    "Value": 10,
                    "Message": "Hello"
                }
            },
            {
                "pointer": {
                    "Value": 10,
                    "Message": "Hello"
                }
            }
        ],
        "Items": [
            {
                "Value": 10,
                "Message": "Hello"
            },
            {
                "Value": 10,
                "Message": "Hello"
            }
        ],
        "Numbers": [
            1,
            2,
            3
        ],
        "Empty": []
    }
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(JsonIntrospectionWithNestedUnwrapping)
        {
            std::stringstream ss;
            {
                garbage::nested_data d{};

                mira::introspector<cereal::JSONOutputArchive> intro{ ss };
                intro(d);
            }

            std::string expected =
R"({
    "value0": {
        "Value": {
            "pointer": {
                "Value": 10,
                "Message": "Hello"
            }
        },
        "Message": "Hello"
    }
})";

            Assert::AreEqual(expected, ss.str());
        }

        TEST_METHOD(TestSimpleXRay)
        {
            const int32_t input1 = 1, input2 = 2;
            int32_t output = -1;

            std::stringstream actual;
            {
                XRAY_FUNCTION("TEST",
                    XR_INPUT(input1, input2)
                    XR_OUTPUT(output),
                    actual
                );

                output = input1 + input2;
            }

            std::stringstream expected;
            expected <<
R"({
    ")" << __FUNCTION__ << R"(": {
        "Input": {
            "value0": 1,
            "value1": 2
        },
        "Output": {
            "value0": 3
        }
    }
})";

            Assert::AreEqual(expected.str(), actual.str());
        }

        TEST_METHOD(TestXRayEmptyArray)
        {
            int32_t intput1 = 1;
            gsl::span<int32_t> input2{};

            std::stringstream actual;
            {
                XRAY_FUNCTION("TEST",
                    XR_INPUT(intput1, input2),
                    actual
                );
            }

            std::stringstream expected;
            expected <<
R"({
    ")" << __FUNCTION__ << R"(": {
        "Input": {
            "value0": 1,
            "value1": []
        },
        "Output": {}
    }
})";

            Assert::AreEqual(expected.str(), actual.str());
        }

        struct MyEvent
        {
            std::string Name;
            std::chrono::seconds Seconds;
        };


        TEST_METHOD(GivenAListener_SendingAnObject_GetsReceived)
        {
            MyEvent received{};

            auto listener = mira::object_trace::listen<MyEvent>("messages", [&](std::intptr_t instance, const MyEvent& data)
            {
                Assert::AreEqual<std::intptr_t>(0, instance);
                received = data;
            });

            FIRE_OBJECT_TRACE("messages", nullptr, (MyEvent{ "Hazaaaaah!", std::chrono::seconds{ 22 } }));

#ifndef NDEBUG
            Assert::AreEqual<std::string>("Hazaaaaah!", received.Name);
            Assert::AreEqual(22, (int)received.Seconds.count(), L"Invalid Count");
#else
            Assert::AreEqual<std::string>("", received.Name);
            Assert::AreEqual(0, (int)received.Seconds.count(), L"Invalid Count");
#endif
        }

        TEST_METHOD(GivenAnUnregisteredListener_SendingAnObject_DoesntGetReceived)
        {
            MyEvent received{};

            {
                auto listener = mira::object_trace::listen<MyEvent>("messages", [&](std::intptr_t instance, const MyEvent& data)
                {
                    Assert::AreEqual<std::intptr_t>(0, instance);
                    received = data;
                });
            }

            FIRE_OBJECT_TRACE("messages", nullptr, (MyEvent{ "Hazaaaaah!", std::chrono::seconds{ 22 } }));

            Assert::AreEqual<std::string>("", received.Name);
            Assert::AreEqual(0, (int)received.Seconds.count(), L"Invalid Count");
        }

        TEST_METHOD(GivenAnotherListener_SendingAnObject_DoesntGetReceived)
        {
            MyEvent received{};

            {
                auto listener = mira::object_trace::listen<MyEvent>("message1", [&](std::intptr_t instance, const MyEvent& data)
                {
                    Assert::AreEqual<std::intptr_t>(0, instance);
                    received = data;
                });
            }

            FIRE_OBJECT_TRACE("message2", nullptr, (MyEvent{ "Hazaaaaah!", std::chrono::seconds{ 22 } }));

            Assert::AreEqual<std::string>("", received.Name);
            Assert::AreEqual(0, (int)received.Seconds.count(), L"Invalid Count");
        }

        TEST_METHOD(GivenAWildcardListener_SendingAnObject_GetsReceived)
        {
            MyEvent received{};
            std::string source{};

            auto listener = mira::object_trace::listen<MyEvent>([&](const char* channel, std::intptr_t instance, const MyEvent& data)
            {
                Assert::AreEqual<std::intptr_t>(0xabc, instance);
                received = data;
                source = channel;
            });

            FIRE_OBJECT_TRACE("messages", (void*)0xabc, (MyEvent{ "Hazaaaaah!", std::chrono::seconds{ 22 } }));

#ifndef NDEBUG
            Assert::AreEqual<std::string>("Hazaaaaah!", received.Name);
            Assert::AreEqual(22, (int)received.Seconds.count(), L"Invalid Count");
            Assert::AreEqual<std::string>("messages", source);
#else
            Assert::AreEqual<std::string>("", received.Name);
            Assert::AreEqual(0, (int)received.Seconds.count(), L"Invalid Count");
            Assert::AreEqual<std::string>("", source);
#endif
        }
    };
}
