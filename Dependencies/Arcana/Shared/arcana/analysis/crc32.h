// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <stddef.h>  // for size_t
#include <stdint.h>  // for int32_t
#include <memory>    // for unique_ptr

namespace mira
{
    int32_t crc32(const void* data, size_t size);

    struct continuous_crc32
    {
        continuous_crc32();
        ~continuous_crc32();

        continuous_crc32(continuous_crc32&&);
        continuous_crc32& operator=(continuous_crc32&&);

        void process(const void* data, size_t size);

        int32_t checksum() const;

    private:
        struct impl;
        std::unique_ptr<impl> m_impl;
    };
}
