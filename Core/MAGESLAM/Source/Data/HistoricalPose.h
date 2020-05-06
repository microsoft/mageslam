// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Device/CameraCalibration.h"
#include "Data\Types.h"
#include "Data\Operators.h"
#include "Mapping\MapPointKeyframeAssociations.h"
#include "Utils\cv.h"
#include "arcana\containers\sorted_vector.h"
#include "Proxies\Proxy.h"
#include "Proxies\KeyframeFields.h"
#include "opencv2\core\core.hpp"
#include "Data\InternalDepth.h"

#include <chrono>

// forward-declare the friend class method for accessing root objects in unit tests
namespace UnitTests
{
    class HistoricalPoseUnitTest;
}

namespace mage
{
    class Keyframe;
    class PoseHistory;

    // Eigen is 16 byte aligned (hard coded in Eigen source)
    struct alignas(16) FrameWorldPosition
    {
        mage::Quaternion    Rotation;
        cv::Vec3f           Position;
    };

    inline FrameWorldPosition ToWorldPosition(const mage::Pose& pose)
    {
        const cv::Matx44f worldMat = pose.GetInverseViewMatrix();
        return FrameWorldPosition{ ToQuat(Rotation(worldMat)), Translation(worldMat) };
    };

    // converts from the world position representation to the Pose view matrix representation
    inline Pose ToPose(const FrameWorldPosition& worldPosition)
    {
        cv::Matx33f rotation = ToMat(worldPosition.Rotation);

        // pose expects a world matrix
        return Pose(
        {   rotation(0,0),rotation(0,1),rotation(0,2),worldPosition.Position(0),
            rotation(1,0),rotation(1,1),rotation(1,2),worldPosition.Position(1),
            rotation(2,0),rotation(2,1),rotation(2,2),worldPosition.Position(2),
            0,0,0,1 }
        );
    };

    struct HistoricalPoseOffset
    {
        FrameWorldPosition FramePosition;
        Id<Keyframe> KeyframeId;

        struct less
        {
            bool operator()(const HistoricalPoseOffset& left, const HistoricalPoseOffset& right) const
            {
                return left.KeyframeId < right.KeyframeId;
            }

            bool operator()(const HistoricalPoseOffset& left, const Id<Keyframe>& right) const
            {
                return left.KeyframeId < right;
            }
        };

        const mage::Quaternion& RotationOffset() const
        {
            return FramePosition.Rotation;
        };

        const cv::Vec3f& PositionOffset() const
        {
            return FramePosition.Position;
        };

        bool operator==(const HistoricalPoseOffset& other) const
        {
            return KeyframeId == other.KeyframeId;
        }

        bool operator==(const Id<Keyframe>& other) const
        {
            return KeyframeId == other;
        }

    };

    using HistoricalPoseOffsetVector = mira::sorted_vector <HistoricalPoseOffset, HistoricalPoseOffset::less, Eigen::aligned_allocator<HistoricalPoseOffset>>;
    using FramePositionRefPair = std::pair<const FrameWorldPosition&, const HistoricalPoseOffset&>;

    class HistoricalPose
    {
        friend class ::UnitTests::HistoricalPoseUnitTest;

    public:
        HistoricalPose(const Id<Keyframe> keyframeId, const FrameId& frameId, const CameraCalibration& distortedCalibration, InternalDepth depth);

        const FrameId& GetFrameId() const
        {
            return m_frameId;
        }

        const CameraCalibration& GetDistortedCalibration() const
        {
            return m_distortedCalibration;
        }

        const Id<Keyframe>& GetId() const
        {
            return m_keyframeId;
        }

        Depth GetDepth() const;

        void ConnectToKeyframe(const HistoricalPoseOffset& historicalPose);

        void DisconnectFromKeyframe(Id<Keyframe> keyframeId);

        void DisconnectFromAllKeyframes();

        const HistoricalPoseOffsetVector& GetKeyframeOffsets() const
        {
            return m_keyframeMappings;
        }

        // given the set of offsets and positions, compute a new position for this point
        static FrameWorldPosition ComputeWorldPosition(gsl::span<const FramePositionRefPair> positions);

        static FrameWorldPosition ComputeOffsetPosition(const FramePositionRefPair& pair);

        static HistoricalPoseOffset ComputeFrameOffset(const FrameWorldPosition& historicalPose, const FrameWorldPosition& keyframePose, const Id<Keyframe> keyframeId);

        struct less
        {
            bool operator()(const HistoricalPose& left, const HistoricalPose& right) const
            {
                return left.m_frameId < right.m_frameId;
            }

            bool operator()(const HistoricalPose& left, const FrameId& right) const
            {
                return left.m_frameId < right;
            }
        };

        bool operator==(const HistoricalPose& other) const
        {
            return m_frameId == other.m_frameId;
        }

        bool operator==(const FrameId& other) const
        {
            return m_frameId == other;
        }

    private:
        const FrameId m_frameId;
        // TODO remove this following change to MAGESLAM to use the frameId instead of generating Id<Keyframe> things
        const Id<Keyframe> m_keyframeId; // mage ID for this frame
        const CameraCalibration m_distortedCalibration; // calibration for this frame

        InternalDepth m_depth; // Near and far plane depths
        HistoricalPoseOffsetVector m_keyframeMappings; // vector of keyframes close to this tracking frame
    };
    
}
