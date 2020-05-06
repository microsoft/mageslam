// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <stdint.h>
#include <chrono>
#include <arcana/analysis/introspector.h>

#include "Utils\id_generator.h"

namespace mage
{
    using IdT = int32_t;

    template<typename ScopeT>
    using IdGenerator = id_generator<IdT, ScopeT>;

    template<typename ScopeT>
    using Id = typename id<IdT, ScopeT>;

    using KeypointDescriptorIndex = std::size_t;
}
