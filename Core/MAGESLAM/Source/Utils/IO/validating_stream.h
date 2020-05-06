// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <exception>
#include <type_traits>

namespace mage
{
    template<typename StreamT>
    struct validating_istream
    {
        validating_istream(StreamT stream)
            : m_stream{ std::move(stream) }
        {}

        template<typename ArgT>
        validating_istream& operator >>(ArgT& arg)
        {
            m_stream >> arg;
            if (m_stream.fail())
                throw std::runtime_error("failed to read parameter");

            return *this;
        }

        StreamT* operator ->()
        {
            return &m_stream;
        }

    private:
        StreamT m_stream;
    };

    template<typename StreamT>
    auto validated_input(StreamT stream)
    {
        return validating_istream<std::decay_t<StreamT>>{ std::move(stream) };
    }
}