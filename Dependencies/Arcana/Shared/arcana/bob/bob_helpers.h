// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    namespace bob
    {
        inline void throw_invalid_version()
        {
            throw std::runtime_error("invalid version");
        }

        template<uint32_t VERSION>
        class empty_context
        {
        public:

            template<class Archive>
            void serialize(Archive&, uint32_t version)
            {
                if (version != VERSION)
                {
                    throw_invalid_version();
                }
            }
        };
    }
}

