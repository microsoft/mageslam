// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "ThreadSafePoseHistory.h"
#include "Utils\Logging.h"
#include "Utils\thread_memory.h"

using namespace std;

namespace mage
{
    struct ThreadSafePoseHistoryTempData
    {
        ThreadSafePoseHistoryTempData::ThreadSafePoseHistoryTempData(const KeyframeProxy& keyframeProxy, gsl::span<const KeyframeReprojection> covisibleKeyframes, InternalDepth depth, thread_memory memory)
            :   frame{ keyframeProxy },
                depth{ std::move(depth) }
        {
            SCOPE_TIMER(ThreadSafePoseHistoryTempData::ThreadSafePoseHistoryTempData);
            auto mapPoints = memory.stack_vector<MapPointTrackingProxy>(keyframeProxy.GetMapPointCount());
            keyframeProxy.GetMapPoints(mapPoints);

            for (const auto& data : covisibleKeyframes)
            {
                unsigned int countPoints = set_intersection_count(
                    data.Points.begin(), data.Points.end(),
                    mapPoints.begin(), mapPoints.end());

                covisibleKeyframesData.emplace(countPoints, Proxy<Keyframe, proxy::Pose>{data.KeyframeId, data.Pose});
            }
        }

        std::multimap<unsigned int,Proxy<Keyframe, proxy::Pose>> covisibleKeyframesData;
        Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics> frame;
        InternalDepth depth;
    };

    
    ThreadSafePoseHistory::ThreadSafePoseHistory(const PoseHistorySettings& settings)
        : m_poseHistory{ std::make_unique<PoseHistory>(settings) },
        m_temporaryHistoricalPoses{ make_unique<std::deque<ThreadSafePoseHistoryTempData>>()}
    {}

    ThreadSafePoseHistory::~ThreadSafePoseHistory()
    {}

    std::unique_ptr<PoseHistory> ThreadSafePoseHistory::Release(std::unique_ptr<ThreadSafePoseHistory> history)
    {
        return std::move(history->m_poseHistory);
    }

    void ThreadSafePoseHistory::AddKeyframeToTrackingHistory(const Id<Keyframe> keyframeId, const Pose& pose)
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };

        m_poseHistory->KeyframeAdded(keyframeId, pose);
    }

    void ThreadSafePoseHistory::ConnectAdjustedKeyframeToNewlyEstimatedPoses(const Id<Keyframe> keyframeId, const Pose& pose)
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };

        m_poseHistory->ConnectAdjustedKeyframeToNewlyEstimatedPoses(keyframeId, pose);
    }

    void ThreadSafePoseHistory::RemoveKeyframeFromTrackingHistory(const Id<Keyframe> keyframeId, gsl::span<const Id<Keyframe>> covisibleKeyframes)
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };

        m_poseHistory->KeyframeRemoved(keyframeId, covisibleKeyframes);
    }

    void ThreadSafePoseHistory::UpdateKeyframeHistoryPositionUnsafe(const Id<Keyframe> keyframeId, const Pose& pose)
    {
        m_poseHistory->UpdateKeyframePose(keyframeId, pose);
    }

    void ThreadSafePoseHistory::AdjustPoses(const std::vector<Proxy<Keyframe, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints>>& adjustmentData)
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };

        for (const auto& keyframe : adjustmentData)
        {
            UpdateKeyframeHistoryPositionUnsafe(keyframe.GetId(), keyframe.GetPose());
        }
    }

    void ThreadSafePoseHistory::AddHistoricalPose(const KeyframeProxy& keyframeProxy, gsl::span<const KeyframeReprojection> covisibleKeyframes, InternalDepth depth, thread_memory memory)
    {
        SCOPE_TIMER(ThreadSafePoseHistory::AddHistoricalPose);

        assert(covisibleKeyframes.size() > 0 && "Must have some covisible keyframes");

        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };

        // only add to the temporary history maintained by this class
        // add to the real pose history whenever there is a data update for it
        m_temporaryHistoricalPoses->emplace_back(keyframeProxy, covisibleKeyframes, std::move(depth), memory);
    }

    vector<boost::optional<PoseHistory::TrackingInformation>> ThreadSafePoseHistory::GetTrackingInformationForFrames(const gsl::span<const FrameId> frameIds) const
    {
        shared_lock<shared_mutex> lock{ m_poseHistory_mutex };

        vector<boost::optional<PoseHistory::TrackingInformation>> frames;
        frames.reserve(frameIds.size());

        for (const FrameId& frameId : frameIds)
        {
            // first search in the temporary history to see if a match is found, the size of the temporary history should be small, no larger than 10-15 elements
            const auto& foundElement = std::find_if(m_temporaryHistoricalPoses->begin(), m_temporaryHistoricalPoses->end(), [&frameId](const ThreadSafePoseHistoryTempData& hp)
            {
                return (frameId == hp.frame.GetAnalyzedImage()->GetFrameId());
            });

            // found frame in recent, return it.
            if (foundElement != m_temporaryHistoricalPoses->end())
            {
                frames.emplace_back(PoseHistory::TrackingInformation{
                    frameId,
                    foundElement->frame.GetPose(), 
                    foundElement->frame.GetAnalyzedImage()->GetDistortedCalibration().CreateCameraModel(),
                    foundElement->depth.AsDepth()
                });
            }
            else
            {
                frames.push_back(m_poseHistory->GetTrackingInformationForFrame(frameId));
            }
        }

        return frames;
    }

    void ThreadSafePoseHistory::FlushTemporaryPoseHistory()
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };

        while (!m_temporaryHistoricalPoses->empty())
        {
            ThreadSafePoseHistoryTempData& data = m_temporaryHistoricalPoses->front();
            m_poseHistory->AddHistoricalPose(data.frame,data.covisibleKeyframesData, std::move(data.depth));
            m_temporaryHistoricalPoses->pop_front();
        }
    }

    bool ThreadSafePoseHistory::TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest, const VolumeOfInterestSettings& voiSettings) const
    {
        SCOPE_TIMER(ThreadSafePoseHistory::GetVolumeOfInterest);
        shared_lock<shared_mutex> lock{ m_poseHistory_mutex };

        return m_poseHistory->TryGetVolumeOfInterest(volumeOfInterest, voiSettings);
    }

    void ThreadSafePoseHistory::Clear()
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };
        m_poseHistory->Clear();

        m_temporaryHistoricalPoses->clear();
    }

    void ThreadSafePoseHistory::DebugGetAllPoses(std::vector<Pose>& poses) const
    {
        unique_lock<shared_mutex> lock{ m_poseHistory_mutex };
        m_poseHistory->DebugGetAllPoses(poses);
    }

}
