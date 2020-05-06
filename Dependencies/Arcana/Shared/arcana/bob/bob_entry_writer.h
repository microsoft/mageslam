// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "bob_writer.h"

#include <arcana/streams/memory_device.h>

#include <assert.h>
#include <sstream>

#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>

#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/positioning.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/optional.hpp>
#include <iosfwd>

namespace mira
{
    struct bob_empty_context
    {
        template<typename ArchiveT>
        void serialize(ArchiveT&, std::uint32_t)
        {}
    };

    template<typename EntryT>
    class bob_entry_writer
    {
    public:
        using context_t = typename EntryT::context_t;

        bob_entry_writer(bob_writer& io, const std::string& name, context_t context, size_t chunkSize = 10)
            : m_io{ io }
            , m_entryDescriptor{ io.file().get_stream(name) }
            , m_chunkSize{ chunkSize }
            , m_context{ std::move(context) }
        {
            m_memory.resize(2 * chunkSize * sizeof(EntryT));

            {
                std::stringstream stream(std::ios::binary | std::ios::out | std::ios::in);
                stream.precision(std::numeric_limits<float>::max_digits10);

                write_block(
                    stream, std::numeric_limits<std::int64_t>::lowest(), [&](cereal::PortableBinaryOutputArchive&) {
                        {
                            cereal::JSONOutputArchive json{ stream };

                            json(cereal::make_nvp("_Stream", m_entryDescriptor.Entry.name()));
                            json(cereal::make_nvp("_Version", m_entryDescriptor.Entry.version()));

                            m_context.serialize(json, m_entryDescriptor.Entry.version());
                        }
                        // write an explicit null termination to
                        // ensure that the json parser doesn't read
                        // the following binary blob as json.
                        stream.write("\0", 1);
                    });

                m_io.write(stream);
            }
        }

        ~bob_entry_writer()
        {
            flush();
        }

        const std::int32_t& entry_idx() const
        {
            return m_entryDescriptor.EntryIdx;
        }

        void write(const std::int64_t& timestamp, const EntryT& element)
        {
            m_currentChunk++;

            if (m_currentChunk >= m_chunkSize)
            {
                flush();
            }

            boost::iostreams::stream<memory_device> stream{ memory_device{ m_memory, m_pos },
                                                            std::ios::binary | std::ios::out | std::ios::in };

            write_block(stream, timestamp, [this, &element](cereal::PortableBinaryOutputArchive& ar) {
                const_cast<EntryT&>(element).serialize(ar, m_entryDescriptor.Entry.version());
            });

            m_pos = stream.tellp();
        }

        void flush()
        {
            if (m_pos == 0ll)
                return;

            m_io.write(m_memory.data(), gsl::narrow_cast<size_t>(m_pos));

            m_pos = 0;
        }

    private:
        template<typename CallableT>
        void write_block(std::ostream& stream, const std::int64_t& timestamp, CallableT&& callable)
        {
            std::streamoff startOfWrite = stream.tellp();

            cereal::PortableBinaryOutputArchive ar{ stream };

            assert(m_prevTimestamp <= timestamp && "Out of order timestamp passed to bob entry writer");
            m_prevTimestamp = timestamp;

            // we start off by writing an entry for the element
            // but we leave out the next block offset because we don't
            // know how big the data we're going to serialize is
            std::streamoff startOfBlock = stream.tellp();

            bob::entry_instance block{ timestamp, 0, entry_idx() };

            block.serialize(ar, bob::VERSION);

#ifndef NDEBUG
            std::streamoff endOfBlock = stream.tellp();
#endif

            // now we save the data
            callable(ar);

            // check how large the data was
            std::streamoff endOfData = stream.tellp();

            // update the next block offset in our header
            block.NextBlockOffset = endOfData - startOfWrite;

            // go back to where we wrote it
            stream.seekp(startOfBlock);

            // and overwrite it in order to have the right value for the NextBlockOffset property
            block.serialize(ar, bob::VERSION);

            assert(stream.tellp() == endOfBlock);

            // then seek back to the end of the data
            stream.seekp(endOfData);
        }

        bob_writer& m_io;
        const bob_stream_file::entry_descriptor m_entryDescriptor;
        const size_t m_chunkSize;
        size_t m_currentChunk{};

        std::vector<char> m_memory;
        std::streamoff m_pos{ 0 };

        // constant data for the stream
        context_t m_context;

        std::int64_t m_prevTimestamp{std::numeric_limits<std::int64_t>::lowest()};
    };
}
