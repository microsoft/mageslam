// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\HistoricalPose.h"
#include "MageSettings.h"
#include "boost\format.hpp"

// forward-declare the friend class method for accessing root objects in unit tests
namespace UnitTests
{
    class HistoricalPoseUnitTest;
}

namespace mage
{
    // special data stored only for tracking frames which are currently treated as keyframes
    struct PoseHistoryKeyframe
    {
        Id<Keyframe>                  KeyframeId;
        FrameWorldPosition            WorldPosition;
        std::set <Id<Keyframe>>       Connections;          // the HistoricalPoses that are connected to this Keyframe

        struct less
        {
            bool operator()(const PoseHistoryKeyframe& left, const PoseHistoryKeyframe& right) const
            {
                return left.KeyframeId < right.KeyframeId;
            }

            bool operator()(const PoseHistoryKeyframe& left, const Id<Keyframe>& right) const
            {
                return left.KeyframeId < right;
            }
        };

        bool operator==(const PoseHistoryKeyframe& other) const
        {
            return KeyframeId == other.KeyframeId;
        }

        bool operator==(const Id<Keyframe>& other) const
        {
            return KeyframeId == other;
        }
    };

    class PoseHistory
    {
        friend class ::UnitTests::HistoricalPoseUnitTest;

    public:
        struct TrackingInformation
        {
            FrameId Id;
            Pose Pose;
            std::shared_ptr<const calibration::CameraModel> CameraModel;
            Depth Depth;

            TrackingInformation(const FrameId& id, const mage::Pose& pose, std::shared_ptr<const calibration::CameraModel> cameraModel, const mage::Depth& depth)
                : Id{ id }, Pose{ pose }, CameraModel{ cameraModel }, Depth{ depth }
            {}
        };

        PoseHistory(const mage::PoseHistorySettings& trackingHistorySettings);

        void AddHistoricalPose(const Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics>::ViewT& frame, const std::multimap<unsigned int, Proxy<Keyframe, proxy::Pose>>& covisibleKeyframes, InternalDepth depth);

        boost::optional<TrackingInformation> GetTrackingInformationForFrame(const FrameId& frameId) const;

        // call this method when the position of a keyframe has been adjusted
        void UpdateKeyframePose(const Id<Keyframe>& keyframeId, const Pose& keyframePose);

        // call this method when a keyframe has been culled and is no longer a keyframe
        void KeyframeRemoved(const Id<Keyframe>& keyframeId, gsl::span<const Id<Keyframe>> similarKeyframes);

        // call this method when a new keyframe is added into the map
        void KeyframeAdded(const Id<Keyframe>& keyframeId, const Pose& pose);

        // call this method when a new keyframe is added into the map and finished localBA for the first time.
        void ConnectAdjustedKeyframeToNewlyEstimatedPoses(const Id<Keyframe>& keyframeId, const Pose& pose);

        // reset all the data
        void Clear();

        bool TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest, const VolumeOfInterestSettings& voiSettings) const;

        // debug function to retrieve all the poses in the history
        void DebugGetAllPoses(std::vector<Pose>& poses) const;

        const mira::sorted_vector<HistoricalPose, HistoricalPose::less>& GetHistoricalPoses() const
        {
            return m_historicalPoses;
        }

    private:
        using historical_pose_vector = mira::sorted_vector<HistoricalPose, HistoricalPose::less>;

        void AddHistoricalPose(HistoricalPose historicalPose);
        FrameWorldPosition ComputePoseForFrame(const HistoricalPose& historicalPose) const;

        historical_pose_vector::iterator FindByKeyframeId(const Id<Keyframe>& keyframeId);

        const mage::PoseHistorySettings m_poseHistorySettings;
        historical_pose_vector m_historicalPoses;
        mira::sorted_vector<PoseHistoryKeyframe, PoseHistoryKeyframe::less, Eigen::aligned_allocator<PoseHistoryKeyframe>> m_keyframes;
    };
}
