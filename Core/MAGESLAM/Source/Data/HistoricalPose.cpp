// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "HistoricalPose.h"

namespace mage
{
    HistoricalPose::HistoricalPose(const Id<Keyframe> keyframeId, const FrameId& frameId, const CameraCalibration& cameraSettings, InternalDepth depth)
        :   m_frameId(frameId),
            m_distortedCalibration(cameraSettings),
            m_keyframeId(keyframeId),
            m_depth(std::move(depth))
    {
    }

    Depth HistoricalPose::GetDepth() const
    {
        return m_depth.AsDepth();
    }

    void HistoricalPose::ConnectToKeyframe(const HistoricalPoseOffset& historicalPose)
    {
        // we want to ensure that we aren't already connected to this keyframe
        assert(m_keyframeMappings.find(historicalPose.KeyframeId) == m_keyframeMappings.end());
        
        // it is invalid to link to ourselves
        assert(historicalPose.KeyframeId != m_keyframeId);

        m_keyframeMappings.insert(historicalPose);

        // we want to ensure that we successfully inserted the mapping
        assert(m_keyframeMappings.find(historicalPose.KeyframeId) != m_keyframeMappings.end());
    }

    void HistoricalPose::DisconnectFromKeyframe(Id<Keyframe> keyframeId)
    {
        // we want to ensure that we are already connected to this keyframe
        assert(m_keyframeMappings.find(keyframeId) != m_keyframeMappings.end());

        m_keyframeMappings.erase(m_keyframeMappings.find(keyframeId));

        // we want to ensure that we successfully removed the mapping
        assert(m_keyframeMappings.find(keyframeId) == m_keyframeMappings.end());
    }

    void HistoricalPose::DisconnectFromAllKeyframes()
    {
        m_keyframeMappings.clear();
        assert(m_keyframeMappings.size() == 0);
    }

    HistoricalPoseOffset HistoricalPose::ComputeFrameOffset(const FrameWorldPosition& historicalPosePosition, const FrameWorldPosition& keyframePosition, const Id<Keyframe> keyframeId)
    {
        // compute the offset to this frame and return set it up as a tracking frame
        cv::Point3f positionOffset = historicalPosePosition.Position - keyframePosition.Position;

        const mage::Quaternion invRotation = keyframePosition.Rotation.inverse();
        const cv::Matx33f worldRotationInv = ToMat(invRotation);
        const cv::Vec3f position = worldRotationInv * positionOffset;

        mage::Quaternion rotationOffset = invRotation * historicalPosePosition.Rotation;

        return{ { rotationOffset, position } , keyframeId };
    }

    FrameWorldPosition HistoricalPose::ComputeOffsetPosition(const FramePositionRefPair& pair)
    {
        mage::Quaternion rotation = pair.first.Rotation * pair.second.RotationOffset();

        const cv::Matx44f worldPos = FromQuatAndTrans(pair.first.Rotation, pair.first.Position);
        const cv::Vec4f position = worldPos * cv::Vec4f{ pair.second.PositionOffset()[0], pair.second.PositionOffset()[1], pair.second.PositionOffset()[2], 1.f };
        assert(abs(position[3] - 1) < 0.00001f);

        return{ rotation, { position[0], position[1], position[2] } };
    }

    FrameWorldPosition HistoricalPose::ComputeWorldPosition(gsl::span<const FramePositionRefPair> positions)
    {
        assert(positions.size() > 0);

        const float scaleFudge = 0.00001f;
        // compute the total scale which will be useful for scaling the results
        const FrameWorldPosition firstOffsetPosition = ComputeOffsetPosition(positions[0]);

        //weight the contribution from all the connected poses proportional to their distance.
        float proportionSum = 1 / (scaleFudge + sqrt(positions[0].second.PositionOffset().dot(positions[0].second.PositionOffset())));
        cv::Vec3f currentPosition = firstOffsetPosition.Position;
        currentPosition *= proportionSum;

        // we'll compute the 'average' of the quaternion according to the technique described here, but weigh them according to distance
        // http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors

        float w = firstOffsetPosition.Rotation.w() * proportionSum;
        float x = firstOffsetPosition.Rotation.x() * proportionSum;
        float y = firstOffsetPosition.Rotation.y() * proportionSum;
        float z = firstOffsetPosition.Rotation.z() * proportionSum;

        // LERP in each point
        for (ptrdiff_t index = 1; index < positions.size(); index++)
        {
            FrameWorldPosition computedOffsetPosition = ComputeOffsetPosition(positions[index]);

            float weight = 1 / (scaleFudge + sqrt(positions[index].second.PositionOffset().dot(positions[index].second.PositionOffset())));
            currentPosition += computedOffsetPosition.Position * weight;
            
            proportionSum += weight;

            // check to see if the new quaternion is close to the original quaternion, we can inverse the sign if it isn't
            float quatDot = firstOffsetPosition.Rotation.dot(computedOffsetPosition.Rotation);
            if (quatDot < 0.0f)
            {
                // need to flip the sign of the quaternion
                computedOffsetPosition.Rotation = { -computedOffsetPosition.Rotation.w(), -computedOffsetPosition.Rotation.x() , -computedOffsetPosition.Rotation.y() , -computedOffsetPosition.Rotation.z() };
            }

            w += computedOffsetPosition.Rotation.w() * weight;
            x += computedOffsetPosition.Rotation.x() * weight;
            y += computedOffsetPosition.Rotation.y() * weight;
            z += computedOffsetPosition.Rotation.z() * weight;
        }

        assert(proportionSum != 0.0f && "invalid proportionsum");
        float scale = 1.f / proportionSum;

        currentPosition *= scale;

        float denom = sqrtf(w*w + x*x + y*y + z*z);
        assert(denom != 0.0f && "invalid denom");
        float normalizeFactor = 1.f / denom;

        mage::Quaternion worldRotation = { w*normalizeFactor, x*normalizeFactor, y*normalizeFactor, z*normalizeFactor };

        return{ worldRotation , currentPosition };
    }
}
