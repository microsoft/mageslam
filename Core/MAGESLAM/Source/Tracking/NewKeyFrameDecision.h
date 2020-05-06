// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "KeyframeBuilder.h"
#include "Map\ThreadSafeMap.h"

#include <memory>

namespace mage
{
    class NewKeyFrameDecision
    {
    public:
        NewKeyFrameDecision(const MageSlamSettings& settings);

        //returns true if this frame should become a keyframe
        bool IsNewKeyFrame(
            bool mappingIsIdle,
            const KeyframeBuilder& keyframe,
            gsl::span<std::reference_wrapper<const KeyframeReprojection>> connectedKeyframes,
            const collection<Proxy<MapPoint>>& mapPoints,
            const bool relocalizationHappenedThisFrame,
            const float minPointDistance);

    private:
        const MageSlamSettings& m_settings;
        unsigned int m_frameCountSinceLastKeyFrame;
        unsigned int m_frameCountSinceLastRelocalization;
    };
}
