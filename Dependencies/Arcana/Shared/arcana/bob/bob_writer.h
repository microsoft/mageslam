// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "bob_stream_file.h"

#include <atomic>
#include <mutex>

namespace mira
{
    //
    // Shared object that writes bob data blobs to a bob stream file.
    // This is the underlying object that bob_entry_writer uses to
    // write it's entries to the streamed file.
    // This should only be used internally by the bob_entry_writer.
    //
    class bob_writer
    {
    public:
        bob_writer(std::unique_ptr<bob_stream_file> file, std::unique_ptr<std::ostream> stream)
            : m_file{ std::move(file) }
            , m_stream{ std::move(stream) }
        {}

        const bob_stream_file& file() const
        {
            return *m_file;
        }

        bob_stream_file& file()
        {
            return *m_file;
        }

    private:
        void write(char* data, size_t size)
        {
            std::lock_guard<std::mutex> lock{ m_mut };
            m_stream->write(data, size);

            assert(m_stream->good());
        }

        void write(std::istream& stream)
        {
            std::lock_guard<std::mutex> lock{ m_mut };
            *m_stream << stream.rdbuf();

            assert(m_stream->good());
        }

        template<typename T>
        friend class bob_entry_writer;

        std::unique_ptr<bob_stream_file> m_file;
        std::unique_ptr<std::ostream> m_stream;
        std::mutex m_mut;
    };
}
