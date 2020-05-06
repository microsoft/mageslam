// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Keyframe.h"
#include "Tracking\KeyframeBuilder.h"
#include "Pose.h"
#include "Map\Map.h"

namespace mage
{
    const float Keyframe::DISABLED_TETHER_DISTANCE = 0.f;

    Keyframe::Keyframe(const KeyframeBuilder& kf_builder)
        :   m_id{ kf_builder.GetId() },
            m_image{ kf_builder.GetAnalyzedImage() },
            m_pose{ kf_builder.GetPose() },
            m_undistortedIntrinsics{ kf_builder.GetUndistortedIntrinsics() },
            m_associations{ m_image },
            m_immortal{ kf_builder.IsImmortal() },
            m_fixed{ kf_builder.IsFixed() },
            m_tethers{ kf_builder.GetTethers() }
    {
        m_image->MarkAsPublished();
    }
}

