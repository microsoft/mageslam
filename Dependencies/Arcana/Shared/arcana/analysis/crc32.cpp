// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "crc32.h"

#include <boost/crc.hpp>

namespace mira
{
    int32_t crc32(const void* data, size_t size)
    {
        boost::crc_32_type computer;
        computer.process_bytes(data, size);
        return computer.checksum();
    }

    struct continuous_crc32::impl
    {
        boost::crc_32_type computer;
    };

    continuous_crc32::continuous_crc32()
        : m_impl{ std::make_unique<impl>() }
    {}

    continuous_crc32::~continuous_crc32() = default;
    continuous_crc32::continuous_crc32(continuous_crc32&&) = default;
    continuous_crc32& continuous_crc32::operator=(continuous_crc32&&) = default;

    void continuous_crc32::process(const void* data, size_t size)
    {
        m_impl->computer.process_bytes(data, size);
    }

    int32_t continuous_crc32::checksum() const
    {
        return m_impl->computer.checksum();
    }
}
