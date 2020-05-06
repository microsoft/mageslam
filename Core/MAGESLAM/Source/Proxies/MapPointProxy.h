// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Proxies\Proxy.h"
#include "Proxies\MapPointFields.h"

#include "Containers\MapPointAssociations.h"

namespace mage
{
    using MapPointProxy = Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics, proxy::ViewingData, proxy::Descriptor>;
    using MapPointTrackingProxy = Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics>;

    extern template MapPointAssociations<Proxy<MapPoint>>;
    extern template MapPointAssociations<MapPointProxy>;
    extern template MapPointAssociations<MapPointProxy> MapPointAssociations<MapPoint const*>::Convert<MapPointProxy>() const;
    extern template MapPointAssociations<Proxy<MapPoint>> MapPointAssociations<MapPoint const*>::Convert<Proxy<MapPoint>>() const;
    extern template MapPointAssociations<Proxy<MapPoint>> MapPointAssociations<MapPointTrackingProxy>::Convert<Proxy<MapPoint>>() const;

    extern template MapPointAssociations<MapPointTrackingProxy>;
}
