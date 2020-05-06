// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <algorithm>
#include <boost/iostreams/categories.hpp>
#include <gsl/gsl>
#include <iosfwd>
#include <vector>

namespace mira
{
    class memory_device
    {
    public:
        using char_type = char;
        using category = boost::iostreams::seekable_device_tag;

        memory_device(std::vector<char>& memory, std::streamoff pos)
            : m_pos{ pos }
            , m_memory{ memory }
        {}

        std::streamoff position() const
        {
            return m_pos;
        }

        void reset()
        {
            m_pos = 0;
        }

        std::streamsize read(char* s, std::streamsize n)
        {
            // Read up to n characters from the underlying data source
            // into the buffer s, returning the number of characters
            // read; return -1 to indicate EOF
            std::streamsize read = std::min<std::streamsize>(m_memory.size() - m_pos, n);
            if (read == 0)
                return -1;

            std::copy(pointer(), pointer() + gsl::narrow_cast<ptrdiff_t>(read), s);
            m_pos += read;

            return read;
        }

        std::streamsize write(const char* s, std::streamsize n)
        {
            // Write up to n characters to the underlying
            // data sink into the buffer s, returning the
            // number of characters written
            std::streamsize written = 0ll;
            if (m_pos != gsl::narrow<std::streamoff>(m_memory.size()))
            {
                written = std::min<std::streamsize>(n, m_memory.size() - m_pos);
                std::copy(s,
                          s + gsl::narrow_cast<ptrdiff_t>(written),
                          m_memory.begin() + gsl::narrow_cast<ptrdiff_t>(m_pos));
                m_pos += written;
            }

            if (written < n)
            {
                m_memory.insert(m_memory.end(), s + written, s + n);
                m_pos = gsl::narrow<std::streamoff>(m_memory.size());
            }

            return n;
        }

        std::streamoff seek(std::streamoff off, std::ios_base::seekdir way)
        {
            // Seek to position off and return the new stream
            // position. The argument way indicates how off is
            // interpretted:
            //    - std::ios_base::beg indicates an offset from the
            //      sequence beginning
            //    - std::ios_base::cur indicates an offset from the
            //      current character position
            //    - std::ios_base::end indicates an offset from the
            //      sequence end

            std::streamoff next;
            if (way == std::ios_base::beg)
            {
                next = off;
            }
            else if (way == std::ios_base::cur)
            {
                next = m_pos + off;
            }
            else if (way == std::ios_base::end)
            {
                next = m_memory.size() + off - 1;
            }
            else
            {
                throw std::ios_base::failure("bad seek direction");
            }

            // Check for errors
            if (next < 0 || next > gsl::narrow<std::streamoff>(m_memory.size()))
            {
                throw std::ios_base::failure("bad seek offset");
            }

            m_pos = next;
            return m_pos;
        }

    private:
        char* pointer()
        {
            return &m_memory[gsl::narrow_cast<ptrdiff_t>(m_pos)];
        }

        std::streamoff m_pos{0};
        std::vector<char>& m_memory;
    };
}
