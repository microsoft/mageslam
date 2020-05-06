// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "MageSettings.h"
#include "Data\InternalDepth.h"

#include "Proxies\KeyframeProxy.h"

namespace mage
{
    InternalDepth CalculateBoundingPlaneDepthsForKeyframe(const KeyframeProxy& keyframe, const BoundingDepthSettings& settings);
}