// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <shared_mutex>

#include "PoseHistory.h"
#include "MappingKeyframe.h"
#include "BundleAdjustment\BundleAdjust.h"

#include <queue>

namespace mage
{
    struct ThreadSafePoseHistoryTempData;

    class ThreadSafePoseHistory
    {
    public:
        ThreadSafePoseHistory(const PoseHistorySettings&);
        ~ThreadSafePoseHistory();

        /*
        Mark a tracking frame as a keyframe in the history
        */
        void AddKeyframeToTrackingHistory(const Id<Keyframe> keyframeId, const Pose& pose);

        /*
        Remove a Keyframe from the tracking history
        */
        void RemoveKeyframeFromTrackingHistory(const Id<Keyframe> keyframeId, gsl::span<const Id<Keyframe>> covisibleKeyframes);

        /*
        Call this method for each tracking frame as it's pose is discovered
        */
        void AddHistoricalPose(const KeyframeProxy& keyframeProxy, gsl::span<const KeyframeReprojection> covisibleKeyframes, InternalDepth depth, thread_memory memory);

        /*
        add keyframe to recently created keyframes
        */
        void ConnectAdjustedKeyframeToNewlyEstimatedPoses(const Id<Keyframe> keyframeId, const Pose& pose);

        /*
        Call this method to retreive an updated pose for the specified frameId
        */
        boost::optional<PoseHistory::TrackingInformation> GetTrackingInformationForFrame(const FrameId frameId) const
        {
            return GetTrackingInformationForFrames({ &frameId, 1 })[0];
        }

        /*
        Call this method to retreive an updated pose for all the specified frameIds
        */
        std::vector<boost::optional<PoseHistory::TrackingInformation>> GetTrackingInformationForFrames(gsl::span<const FrameId> frameIds) const;

        boost::optional<Pose> GetPoseForFrame(const FrameId frameId) const
        {
            auto info = GetTrackingInformationForFrame(frameId);
            if (info.is_initialized())
            {
                return{ info->Pose };
            }
            else
            {
                return boost::none;
            }
        }

        void AdjustPoses(const std::vector<Proxy<Keyframe, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints>>& adjustmentData);

        void Clear();

        /*
        * Method will push the temporary pose history into the real pose history
        */
        void FlushTemporaryPoseHistory();

        bool TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest, const VolumeOfInterestSettings& voiSettings) const;

        /*
        * Clears the internal PoseHistory and returns it for someone to keep using
        * in another context. The ThreadSafePoseHistory is invalid after
        * calling this method.
        */
        static std::unique_ptr<PoseHistory> Release(std::unique_ptr<ThreadSafePoseHistory> history);

        // debug function to retrieve all the poses in the history
        void DebugGetAllPoses(std::vector<Pose>& poses) const;

    private : 
        /*
        Update the position of a keyframe in the tracking history without getting a lock
        */
        void UpdateKeyframeHistoryPositionUnsafe(const Id<Keyframe> keyframeId, const Pose& pose);

        mutable std::shared_mutex m_poseHistory_mutex;
        std::unique_ptr<PoseHistory> m_poseHistory;

        /*
         * Keep around a short list of historical poses which are not yet published into the history.
         * This exists so that we can have controlled commits into the pose history to provide
         * determinism in the threaded operation of mageslam.  This mostly exists because we
         * do not want to allow TrackLocalMap to hold a schedulign block during its bundle adjust
         * and because we do not want to introduce two scheduling blocks within track local map
         */
        const std::unique_ptr<std::deque<ThreadSafePoseHistoryTempData>> m_temporaryHistoricalPoses;
    };
}
