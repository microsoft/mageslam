// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Utils\buffer.h"
#include "arcana\utils\serialization\base_stream.h"

#include <ostream>
#include <iomanip>

namespace mage
{
    /*
        magestream that serializes most values as text, and binary data as blobs
        in order to avoid indian-ness issues when loading/saving on multiple platforms
    */
    template<typename StreamT = std::iostream>
    struct hybrid_magestream : public mira::base_stream
    {
        using stream_t = StreamT;
        constexpr static auto separator = '\n';

        hybrid_magestream(stream_t& stream)
            : m_stream{ stream }
        {
            m_stream.precision(std::numeric_limits<double>::max_digits10);
        }

        /*
            StreamT contract functions.
        */
        template<typename T>
        void read(T& obj)
        {
            m_stream >> obj;
            m_stream.seekg(sizeof(separator), std::ios::cur);
        }

        template<typename T>
        void read(const T* obj, size_t N)
        {
            m_stream.read((char*)obj, N * sizeof(T));
            m_stream.seekg(sizeof(separator), std::ios::cur);
        }

        template<typename T>
        void write(const T& obj)
        {
            m_stream << obj;
            m_stream << separator;
        }

        template<typename T>
        void write(const T* obj, size_t N)
        {
            m_stream.write((char*)obj, N * sizeof(T));
            m_stream << separator;
        }

    private:
        stream_t& m_stream;
    };
}