// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "SensorSample.h"

namespace mage
{
    bool LegacyDebugRead(std::istream& inputFileStream, const uint64_t& binVersion, SensorSample& outSample);
}