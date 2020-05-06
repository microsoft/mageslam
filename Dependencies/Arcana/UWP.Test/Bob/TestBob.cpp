// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "CppUnitTest.h"

#include "arcana\bob\bob.h"
#include <boost\iostreams\device\array.hpp>
#include <cereal\cereal.hpp>
#include <cereal\archives\portable_binary.hpp>

#include <numeric>
#include <algorithm>
#include <future>

using Assert = Microsoft::VisualStudio::CppUnitTestFramework::Assert;

namespace UnitTests
{
    TEST_CLASS(bObUnitTests)
    {
    public:

        struct buffer_t
        {
            std::vector<char> buffer;
        };

        std::unique_ptr<std::iostream> create_stream_on_buffer(buffer_t& buffer)
        {
            
            return std::make_unique<boost::iostreams::stream<mira::memory_device>>(mira::memory_device{ buffer.buffer, 0 });
        }

        struct DataSampleContext
        {
            bool IsEven{ false };

            template<typename ArchiveT>
            void serialize(ArchiveT& archive, const std::uint32_t /*version*/)
            {
                archive(
                    CEREAL_NVP(IsEven)
                );
            }
        };

        struct DataSample
        {
            using context_t = DataSampleContext;

            int Number{};

            DataSample() = default;

            DataSample(int value)
                : Number{ value }
            {}

            template<typename ArchiveT>
            void serialize(ArchiveT& archive, const std::uint32_t /*version*/)
            {
                archive(
                    CEREAL_NVP(Number)
                );
            }
        };


        TEST_METHOD(BobWriter_WhenCreated_CreatesCorrectStreamsFromHeaderData)
        {
            mira::bob::stream_file_header header{ {
                { "ints", 20 },
                { "doubles", 10 }
                } };

            buffer_t buffer{};

            auto output = create_stream_on_buffer(buffer);

            mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };

            Assert::IsTrue(writer.file().contains_stream("ints"));
            Assert::IsTrue(writer.file().contains_stream("doubles"));

            Assert::AreEqual<std::int32_t>(0, writer.file().get_stream("ints").EntryIdx);
            Assert::AreEqual<std::uint32_t>(20u, writer.file().get_stream("ints").Entry.version());

            Assert::AreEqual<std::int32_t>(1, writer.file().get_stream("doubles").EntryIdx);
            Assert::AreEqual<std::uint32_t>(10u, writer.file().get_stream("doubles").Entry.version());

            mira::bob_entry_writer<DataSample> intstream{ writer, "ints",{} };
            Assert::AreEqual<std::int32_t>(0, intstream.entry_idx(), L"int stream should be at index 0");

            mira::bob_entry_writer<DataSample> doublestream{ writer, "doubles",{} };
            Assert::AreEqual<std::int32_t>(1, doublestream.entry_idx(), L"double stream should be at index 1");
        }

        TEST_METHOD(BobReader_WhenSingleStreamWrittenAndSerialized_ReadsBackCorrectDataInTimestampOrder)
        {
            mira::bob::stream_file_header header{ {
                { "entries", 0 }
                } };

            std::vector<DataSample> data(100);
            std::iota(data.begin(), data.end(), 0);

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);

                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };

                mira::bob_entry_writer<DataSample> stream{ writer, "entries",{ true }, sizeof(int) };

                for (size_t i = 0; i < data.size(); ++i)
                {
                    stream.write(i, data[i]);
                }
            }

            std::vector<DataSample> read;

            {
                auto input = create_stream_on_buffer(buffer);

                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                reader.handle<DataSample>("entries", [&](const DataSampleContext& context, const std::int64_t& timestamp, const DataSample& value)
                {
                    Assert::IsTrue(context.IsEven);

                    Assert::AreEqual<int>((int)timestamp, value.Number);
                    read.push_back(value);
                });

                while (reader.advance())
                {
                }
            }

            Assert::AreEqual(data.size(), read.size());

            for (size_t i = 0; i < data.size(); ++i)
            {
                Assert::AreEqual(data[i].Number, read[i].Number);
            }
        }

        TEST_METHOD(BobReader_WhenTwoStreamsWrittenAndSerialized_ReadsBackCorrectDataInTimestampOrder)
        {
            mira::bob::stream_file_header header{ {
                { "even", 0 },
                { "odd", 0 }
                } };

            std::vector<DataSample> data(100);
            std::iota(data.begin(), data.end(), 0);

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);

                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };

                mira::bob_entry_writer<DataSample> even{ writer, "even",{ true } };
                mira::bob_entry_writer<DataSample> odd{ writer, "odd",{ false } };

                for (DataSample i : data)
                {
                    (i.Number % 2 == 0 ? even : odd).write(i.Number, i);
                }
            }

            std::vector<DataSample> evens;
            std::vector<DataSample> odds;

            std::vector<DataSample> received;

            {
                auto input = create_stream_on_buffer(buffer);

                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                reader.handle<DataSample>("even", [&](const DataSampleContext& context, const std::int64_t& /*timestamp*/, const DataSample& value)
                {
                    Assert::IsTrue(context.IsEven);

                    received.push_back(value);

                    evens.push_back(received.back());
                });

                reader.handle<DataSample>("odd", [&](const DataSampleContext& context, const std::int64_t& /*timestamp*/, const DataSample& value)
                {
                    Assert::IsFalse(context.IsEven);

                    received.push_back(value);

                    odds.push_back(received.back());
                });

                while (reader.advance())
                {
                }
            }

            Assert::AreEqual(data.size() / 2, evens.size(), L"Bad object count");
            Assert::AreEqual(data.size() / 2, odds.size(), L"Bad object count");

            Assert::AreEqual(data.size(), received.size());
            for (size_t i = 0; i < data.size(); ++i)
            {
                Assert::AreEqual(data[i].Number, received[i].Number);
            }
        }

        TEST_METHOD(BobReader_WhenTwoStreamsWrittenAndSerializedOutOfOrder_ReadsBackCorrectDataInTimestampOrder)
        {
            mira::bob::stream_file_header header{ {
                { "even", 0 },
                { "odd", 0 }
                } };

            std::vector<DataSample> data(100);
            std::iota(data.begin(), data.end(), 0);

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);

                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };

                mira::bob_entry_writer<DataSample> even{ writer, "even",{ true } };
                mira::bob_entry_writer<DataSample> odd{ writer, "odd",{ false } };

                for (DataSample i : data)
                {
                    if (i.Number % 2 != 0)
                        odd.write(i.Number, i);
                }

                for (DataSample i : data)
                {
                    if (i.Number % 2 == 0)
                        even.write(i.Number, i);
                }
            }

            std::vector<DataSample> evens;
            std::vector<DataSample> odds;

            std::vector<DataSample> received;

            {
                auto input = create_stream_on_buffer(buffer);

                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                reader.handle<DataSample>("even", [&](const DataSampleContext& context, const std::int64_t& /*timestamp*/, const DataSample& value)
                {
                    Assert::IsTrue(context.IsEven);

                    received.push_back(value);

                    evens.push_back(received.back());
                });

                reader.handle<DataSample>("odd", [&](const DataSampleContext& context, const std::int64_t& /*timestamp*/, const DataSample& value)
                {
                    Assert::IsFalse(context.IsEven);

                    received.push_back(value);

                    odds.push_back(received.back());
                });

                while (reader.advance())
                {
                }
            }

            Assert::AreEqual(data.size() / 2, evens.size(), L"Bad object count");
            Assert::AreEqual(data.size() / 2, odds.size(), L"Bad object count");

            Assert::AreEqual(data.size(), received.size());
            for (size_t i = 0; i < data.size(); ++i)
            {
                Assert::AreEqual(data[i].Number, received[i].Number);
            }
        }

        struct Frame
        {
            using context_t = mira::bob_empty_context;

            uint64_t ts;
            char bytes[320 * 240];

            template<typename Archive>
            void serialize(Archive& a, const std::uint32_t /*version*/)
            {
                a(
                    CEREAL_NVP(ts),
                    CEREAL_NVP(bytes)
                );
            }
        };

        struct Pose
        {
            using context_t = mira::bob_empty_context;

            uint64_t id;
            float position[3];
            float rotation[4];

            template<typename Archive>
            void serialize(Archive& a, const std::uint32_t /*version*/)
            {
                a(
                    CEREAL_NVP(id),
                    CEREAL_NVP(position),
                    CEREAL_NVP(rotation)
                );
            }
        };

        TEST_METHOD(BobReader_WhenTwoComplexStreamsWrittenAndSerialized_ReadsBackCorrectDataInTimestampOrder)
        {
            mira::bob::stream_file_header header{ {
                { "frames", 0 },
                { "poses", 0 }
                } };

            std::vector<Frame> frames(100);
            std::vector<Pose> poses(100);

            for (size_t i = 0; i < frames.size(); ++i)
            {
                frames[i].ts = i;
                std::fill(std::begin(frames[i].bytes), std::end(frames[i].bytes), (char)i);

                poses[i].id = i;
                std::fill(std::begin(poses[i].position), std::end(poses[i].position), (float)i);
                std::fill(std::begin(poses[i].rotation), std::end(poses[i].rotation), (float)i);
            }

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);

                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };

                {
                    std::vector<std::future<void>> tasks;
                    tasks.push_back(std::async(std::launch::async, [&]
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds{ rand() % 2 });

                        mira::bob_entry_writer<Frame> framestream{ writer, "frames",{} };

                        for (size_t i = 0; i < frames.size(); ++i)
                        {
                            framestream.write(i, frames[i]);
                        }
                    }));

                    tasks.push_back(std::async(std::launch::async, [&]
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds{ rand() % 2 });

                        mira::bob_entry_writer<Pose> posestream{ writer, "poses",{} };

                        for (size_t i = 0; i < poses.size(); ++i)
                        {
                            posestream.write(i, poses[i]);
                        }
                    }));
                }
            }

            std::vector<Frame> receivedframes;
            std::vector<Pose> receivedposes;

            {
                auto input = create_stream_on_buffer(buffer);

                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                reader.handle<Frame>("frames", [&](const mira::bob_empty_context&, const std::int64_t& timestamp, const Frame& fr)
                {
                    Assert::AreEqual((int)timestamp, (int)fr.ts);
                    receivedframes.push_back(fr);
                });

                reader.handle<Pose>("poses", [&](const mira::bob_empty_context&, const std::int64_t& timestamp, const Pose& po)
                {
                    Assert::AreEqual((int)timestamp, (int)po.id);
                    receivedposes.push_back(po);
                });

                while (reader.advance())
                {
                }
            }

            Assert::AreEqual(frames.size(), receivedframes.size(), L"Bad object count");
            Assert::AreEqual(poses.size(), receivedposes.size(), L"Bad object count");

            for (size_t i = 0; i < frames.size(); ++i)
            {
                Assert::AreEqual<uint64_t>(i, receivedframes[i].ts, L"Bad id");
                Assert::AreEqual<uint64_t>(i, receivedposes[i].id, L"Bad id");
            }
        }

        TEST_METHOD(BobReader_ContainsStream_WithoutStream_ReturnsFalse)
        {
            mira::bob::stream_file_header header{ { { "entries", 0 } } };

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);
                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };
            }

            {
                auto input = create_stream_on_buffer(buffer);
                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                Assert::IsFalse(reader.contains_stream("invalid stream"));
            }
        }

        TEST_METHOD(BobReader_ContainsStream_WithStream_ReturnsTrue)
        {
            mira::bob::stream_file_header header{ { { "entries", 0 } } };

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);
                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };
            }

            {
                auto input = create_stream_on_buffer(buffer);
                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                Assert::IsTrue(reader.contains_stream("entries"));
            }
        }

        TEST_METHOD(BobReader_ContainsHandlerForStream_WithHandler_ReturnsTrue)
        {
            mira::bob::stream_file_header header{ { { "entries", 0 } } };

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);
                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };
            }

            {
                auto input = create_stream_on_buffer(buffer);
                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };
                reader.handle<DataSample>("entries", [](auto, auto, auto) {});

                Assert::IsTrue(reader.contains_handler_for_stream("entries"));
            }
        }

        TEST_METHOD(BobReader_ContainsHandlerForStream_WithoutHandler_ReturnsFalse)
        {
            mira::bob::stream_file_header header{ { { "entries", 0 } } };

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);
                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };
            }

            {
                auto input = create_stream_on_buffer(buffer);
                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                Assert::IsFalse(reader.contains_handler_for_stream("entries"));
            }
        }

        TEST_METHOD(BobReader_ContainsHandlerForStream_WithoutStream_ReturnsFalse)
        {
            mira::bob::stream_file_header header{ { { "entries", 0 } } };

            buffer_t buffer{};

            {
                auto output = create_stream_on_buffer(buffer);
                mira::bob_writer writer{ std::make_unique<mira::bob_stream_file>(header), std::move(output) };
            }

            {
                auto input = create_stream_on_buffer(buffer);
                mira::bob_reader reader{ std::make_unique<mira::bob_stream_file>(header), std::move(input) };

                Assert::IsFalse(reader.contains_handler_for_stream("invalid stream"));
            }
        }
    };
}
