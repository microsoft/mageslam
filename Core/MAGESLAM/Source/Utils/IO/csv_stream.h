// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <exception>
#include <type_traits>
#include <locale>
#include <assert.h>

#include <arcana\type_traits.h>

namespace mage
{
    template<typename StreamT>
    struct csv_istream
    {
        csv_istream(StreamT stream)
            : m_stream{ std::move(stream) }
        {
            m_stream >> std::noskipws;
            m_stream.imbue(std::locale{ m_stream.getloc(), new csv_whitespace{} });
        }

        template<typename ArgT>
        csv_istream& operator >> (ArgT& arg)
        {
            skip_whitespace();

            if (at_seperator() && mira::is_string<std::decay_t<ArgT>>())
            {
                // strings can be of zero length
                arg = std::decay_t<ArgT>{};
            }
            else
            {
                m_stream >> arg;

                if (m_stream.fail())
                    throw std::runtime_error("failed to read parameter");
            }

            skip_whitespace();

            skip_comma();

            return *this;
        }

        StreamT* operator ->()
        {
            return &m_stream;
        }

    private:
        bool at_seperator()
        {
            return m_stream.eof() || m_stream.peek() == ',';
        }

        void skip_comma()
        {
            if (!m_stream.eof() && m_stream.peek() == ',')
                advance();
        }

        void skip_whitespace()
        {
            while (!m_stream.eof() && std::isspace(m_stream.peek()))
            {
                advance();
            }
        }

        void advance()
        {
            m_stream.get();
        }

        struct csv_whitespace : std::ctype<char>
        {
            csv_whitespace(std::size_t refs = 0)
                : ctype(make_table(), false, refs)
            {}

        private:
            static const mask* make_table()
            {
                static std::vector<mask> mask{ classic_table(), classic_table() + table_size };
                mask[','] |= space; // comma will be classified as whitespace
                mask[' '] &= ~space; // space will not be classified as whitespace
                mask['\t'] &= ~space; // tabs will not be classified as whitespace
                return mask.data();
            }
        };

        StreamT m_stream;
    };

    template<typename StreamT>
    auto csv_input(StreamT stream)
    {
        return csv_istream<std::decay_t<StreamT>>{ std::move(stream) };
    }
}