// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <vector>

#include "Proxies\Proxy.h"
#include "Proxies\MapPointFields.h"
#include "Proxies\KeyframeFields.h"

namespace mage
{
    struct MapState
    {
        using MapPointT = Proxy<MapPoint, proxy::Position, proxy::ViewingData, proxy::DescriptorCopy>;
        using KeyframeT = Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics, proxy::Associations<Proxy<MapPoint>>, proxy::PoseConstraints>;

        std::vector<MapPointT> MapPoints;
        std::vector<KeyframeT> Keyframes;
    };
}