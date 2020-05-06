// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Proxies\MapPointProxy.h"
#include "Tracking\KeyframeBuilder.h"

#include <vector>
#include <arcana/analysis/introspector.h>

namespace mage
{
    struct InitializationData
    {
        std::vector<MapPointTrackingProxy> MapPoints;
        std::vector<std::shared_ptr<KeyframeBuilder>> Frames;

        const std::shared_ptr<KeyframeBuilder>& AddKeyframeBuilder(
            const std::shared_ptr<const AnalyzedImage>& frame,
            const Pose& pose)
        {
            Frames.push_back(std::make_shared<KeyframeBuilder>(frame, pose));
            return Frames.back();
        }

        void Clear()
        {
            MapPoints.clear();
            Frames.clear();
        }
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const InitializationData& data)
    {
        intro(
            cereal::make_nvp("MapPoints", data.MapPoints),
            cereal::make_nvp("Frames", data.Frames)
        );
    }
}
