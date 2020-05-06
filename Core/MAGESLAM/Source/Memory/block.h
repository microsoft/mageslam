// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mage
{
    namespace memory
    {
        struct block
        {
            void* memory;
            size_t size;

            block(void* mem, size_t size)
                : memory{ mem }, size{ size }
            {}

            block()
                : memory{ nullptr }, size{ 0 }
            {}

            /*
                Reset a block by setting it to null
            */
            block& operator =(std::nullptr_t)
            {
                memory = nullptr;
                size = 0;
                return *this;
            }

            /*
                Check whether a block has been allocated/initialized properly
            */
            bool operator ==(std::nullptr_t)
            {
                return memory == nullptr;
            }

            bool operator !=(std::nullptr_t)
            {
                return !(*this == nullptr);
            }
        };
    }
}