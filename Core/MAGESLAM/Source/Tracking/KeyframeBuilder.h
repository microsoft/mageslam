// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Image\AnalyzedImage.h"
#include "Data\Types.h"
#include "Data\Keyframe.h"
#include "Data\Pose.h"
#include "Map\MapPoint.h"
#include "Proxies\MapPointProxy.h"
#include "Containers\MapPointAssociations.h"
#include "Proxies\KeyframeProxy.h"

#include <memory>
#include <functional>

namespace mage
{
    class KeyframeBuilder : public KeyframeProxy
    {
    public:
        KeyframeBuilder(
            const std::shared_ptr<const AnalyzedImage>& image,
            const mage::Pose& pose,
            const std::vector<MapPointAssociations<MapPointTrackingProxy>::Association>& mapPoints);
        
        KeyframeBuilder(
            const std::shared_ptr<const AnalyzedImage>& image,
            const mage::Pose& pose);

        // Cannot copy MapPointAssociations, so cannot copy KeyframeBuilder
        KeyframeBuilder& operator=(KeyframeBuilder&&) = delete;
        KeyframeBuilder(KeyframeBuilder&&) = delete;
        KeyframeBuilder operator=(KeyframeBuilder&) = delete;
        KeyframeBuilder(const KeyframeBuilder&) = delete;

        void AddExtrinsicTether(const mage::Id<Keyframe>& originId, const mage::Pose& fromPose, float weight)
        {
            cv::Matx44f transform = fromPose.GetViewMatrix4x4() * GetPose().GetInverseViewMatrix();
            auto rotation = transform.get_minor<3, 3>(0, 0);
            auto translation = transform.col(3);
            KeyframeProxy::AddExtrinsicTether(
                originId,
                { translation(0), translation(1), translation(2) },
                ToQuat(rotation),
                weight);
        }

        void AddDistanceTether(const mage::Id<Keyframe>& originId, float distance, float weight)
        {
            assert(originId < KeyframeProxy::GetId() && "Can only store tethers that refer to earlier keyframes.");
            KeyframeProxy::AddDistanceTether(originId, distance, weight);
        }

        void AddExtrinsicTether(const mage::Id<Keyframe>& originId, const cv::Vec3f& position, const Quaternion& rotation, float weight)
        {
            assert(originId < KeyframeProxy::GetId() && "Can only store tethers that refer to earlier keyframes.");
            KeyframeProxy::AddExtrinsicTether(originId, position, rotation, weight);
        }

        void AddRotationTether(const mage::Id<Keyframe>& originId, const Quaternion& rotation, float weight)
        {
            assert(originId < KeyframeProxy::GetId() && "Can only store tethers that refer to earlier keyframes.");
            KeyframeProxy::AddRotationTether(originId, rotation, weight);
        }


    private:
        static IdGenerator<Keyframe> s_generator;
    };
}
