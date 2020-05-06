// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Proxies\Proxy.h"
#include "Proxies\KeyframeFields.h"

#include "MapPointProxy.h"

namespace mage
{
    using KeyframeProxy = Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics, proxy::Associations<MapPointTrackingProxy>, proxy::PoseConstraints>;
}
