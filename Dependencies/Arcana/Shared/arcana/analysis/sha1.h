// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <array>     // for array
#include <memory>    // for unique_ptr
#include <stddef.h>  // for size_t

namespace mira
{
    std::array<unsigned int, 5> sha1(const void* data, size_t size);

    struct continuous_sha1
    {
    public:
        continuous_sha1();
        ~continuous_sha1();

        continuous_sha1(continuous_sha1&&);
        continuous_sha1& operator=(continuous_sha1&&);

        void process(const void* data, size_t size);

        std::array<unsigned int, 5> get_digest() const;

    private:
        struct impl;
        std::unique_ptr<impl> m_impl;
    };
}
