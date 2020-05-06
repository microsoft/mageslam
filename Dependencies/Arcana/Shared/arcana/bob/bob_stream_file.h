// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <cstdint>
#include <fstream>

#include "bob_data.h"

namespace mira
{
    //
    // Represents a binary bob stream file. This is the binary file
    // that is represented by the stream_file_header and contains all streams
    // of data that are time synchronized.
    class bob_stream_file
    {
    public:
        struct iterator
        {
            std::int32_t EntryIdx;
            std::streamoff Position;
            bob::entry_instance LastRead;
        };

        struct entry_descriptor
        {
            std::int32_t EntryIdx;
            bob::stream_file_header::entry Entry;
        };

        explicit bob_stream_file(const bob::stream_file_header& header)
            : m_header{ header }
        {}

        bool contains_stream(const std::string& name) const
        {
            auto& entries = m_header.entries();
            return std::find_if(entries.begin(), entries.end(), [&](const bob::stream_file_header::entry& entry) {
                       return entry.name() == name;
                   }) != entries.end();
        }

        std::uint32_t version() const
        {
            return m_header.version();
        }

        const std::vector<bob::stream_file_header::entry>& entries() const
        {
            return m_header.entries();
        }

        entry_descriptor get_stream(const std::string& name) const
        {
            auto& entries = m_header.entries();
            auto stream = std::find_if(
                entries.begin(), entries.end(), [&](const bob::stream_file_header::entry& entry) {
                    return entry.name() == name;
                });

            if (stream == entries.end())
                throw std::invalid_argument("stream doesn't exist in this file");

            return { gsl::narrow<std::int32_t>(std::distance(entries.begin(), stream)), *stream };
        }

        std::vector<iterator> get_iterators() const
        {
            std::vector<iterator> itrs;

            std::transform(std::begin(m_header.entries()),
                           std::end(m_header.entries()),
                           std::back_inserter(itrs),
                           [this](const bob::stream_file_header::entry& entry) {
                               auto entryidx = gsl::narrow<std::int32_t>(std::distance(m_header.entries().data(), &entry));
                               return iterator{ entryidx, 0, {} };
                           });

            return itrs;
        }


    private:
        bob::stream_file_header m_header;
    };
}
