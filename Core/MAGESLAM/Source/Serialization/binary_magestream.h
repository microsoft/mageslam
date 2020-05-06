// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Utils\buffer.h"
#include "arcana\utils\serialization\base_stream.h"

#include <ostream>
#include <iomanip>

namespace mage
{
    template<typename StreamT = std::iostream>
    struct binary_magestream : public mira::base_stream
    {
        using stream_t = StreamT;

        binary_magestream(stream_t& stream)
            : m_stream{ stream }
        {}

        /*
            StreamT contract functions.
        */
        template<typename T>
        void read(T& obj)
        {
            m_stream.read((char*)&obj, sizeof(T));
        }

        template<typename T>
        void read(const T* obj, size_t N)
        {
            m_stream.read((char*)obj, N * sizeof(T));
        }

        template<typename T>
        void write(const T& obj)
        {
            m_stream.write((char*)&obj, sizeof(T));
        }

        template<typename T>
        void write(const T* obj, size_t N)
        {
            m_stream.write((char*)obj, N * sizeof(T));
        }

    private:
        stream_t& m_stream;
    };
}