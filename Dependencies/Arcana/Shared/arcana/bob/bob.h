// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "bob_data.h"
#include "bob_entry_writer.h"
#include "bob_reader.h"

#include <cereal/archives/json.hpp>

namespace mira
{
    inline bob::manifest load_bob_manifest(std::istream& stream)
    {
        bob::manifest manif{};
        {
            cereal::JSONInputArchive archive{ stream };

            archive(manif);
        }
        return manif;
    }

    inline void save_bob_manifest(std::ostream& stream, const bob::manifest& manif)
    {
        cereal::JSONOutputArchive archive{ stream };

        archive(const_cast<bob::manifest&>(manif));
    }
}
