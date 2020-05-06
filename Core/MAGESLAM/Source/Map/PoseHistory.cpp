// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "PoseHistory.h"

#include "VolumeOfInterest\VolumeOfInterest.h"

using namespace std;

namespace mage
{
    PoseHistory::PoseHistory(const mage::PoseHistorySettings& poseHistorySettings)
        : m_poseHistorySettings{ poseHistorySettings }
    {
        m_historicalPoses.reserve(m_poseHistorySettings.PoseHistoryInitialSize);
        m_keyframes.reserve(m_poseHistorySettings.KeyframeHistoryInitialSize);
    }

    void PoseHistory::Clear()
    {
        m_historicalPoses.clear();
        m_keyframes.clear();
    }

    void PoseHistory::AddHistoricalPose(const Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics>::ViewT& frame, const std::multimap<unsigned int, Proxy<Keyframe, proxy::Pose>>& covisibleKeyframes, InternalDepth depth)
    {
        assert(covisibleKeyframes.size() > 0 && "Must have connections to some frames");

        HistoricalPose historicalPose(frame->GetId(), frame->GetAnalyzedImage()->GetFrameId(), frame->GetAnalyzedImage()->GetDistortedCalibration(), std::move(depth));
        FrameWorldPosition currentPose = ToWorldPosition(frame->GetPose());

        // this construction leverages the fact that the connected keyframes are already
        // sorted in order of most connected, which is what we want to use as our metric
        for (auto itr = covisibleKeyframes.rbegin(); itr != covisibleKeyframes.rend()  && historicalPose.GetKeyframeOffsets().size() < m_poseHistorySettings.InitalInterpolationConnections; itr++)
        {
            const Id<Keyframe> kfId = itr->second.GetId();

            if (kfId == frame->GetId())
            {
                continue;
            }

            // tracking thread will sometimes create frames from keyframes which were removed by the mapping thread
            // in order to handle this race condition, we need to check if the connection is still a keyframe
            // if it is not, then we just ignore it and take the next one
            auto keyframePointer = m_keyframes.find(kfId);
            if (keyframePointer == m_keyframes.end())
            {
                continue;
            }

            historicalPose.ConnectToKeyframe(HistoricalPose::ComputeFrameOffset(currentPose, ToWorldPosition(itr->second.GetPose()), kfId));
        }

        AddHistoricalPose(std::move(historicalPose));
    }

    bool PoseHistory::TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest, const VolumeOfInterestSettings& voiSettings) const
    {
        // Gather the keyframes to be used for the volume of interest calculations.
        vector<VOIKeyframe> keyframes;
        for (auto itr_keyframe = m_keyframes.begin(); itr_keyframe != m_keyframes.end(); itr_keyframe++)
        {
            auto itr_pose = find_if(m_historicalPoses.begin(), m_historicalPoses.end(),
                [itr_keyframe](const HistoricalPose& frame) { return itr_keyframe->KeyframeId == frame.GetId(); });
            
            // Not all keyframes are available for historical poses (initialization poses)
            if (itr_pose != m_historicalPoses.end())
            {
                keyframes.emplace_back(ToPose(itr_keyframe->WorldPosition), itr_pose->GetDepth(), voiSettings);
            }
        }
        
        return CalculateVolumeOfInterest(keyframes, voiSettings, volumeOfInterest);
    }

    void PoseHistory::AddHistoricalPose(HistoricalPose historicalPose)
    {
        m_historicalPoses.insert_presorted(std::move(historicalPose));
    }

    void PoseHistory::KeyframeRemoved(const Id<Keyframe>& keyframeId, gsl::span<const Id<Keyframe>> similarKeyframes)
    {
        // assert that the keyframe is in the list
        auto keyframePointer = m_keyframes.find(keyframeId);
        assert(keyframePointer != m_keyframes.end());

        // need to iterate over all the frames which reference this keyframe and update their links
        // TODO perf, this iterates over the whole list :( could be faster if the links from keyframes were stored in a structure internal to this class
        // TODO some early profiling shows that this method only accounts for ~1% of the CullLocalKeyframes time, so maybe this perf work can be postponed
        for (HistoricalPose& historicalPose : m_historicalPoses)
        {
            bool frameHasKeyframe = false;

            HistoricalPoseOffset offsetToRemove;
            // discover if this tracking frame cares
            for (const HistoricalPoseOffset& offset : historicalPose.GetKeyframeOffsets())
            {
                if (offset.KeyframeId == keyframeId)
                {
                    offsetToRemove = offset;
                    frameHasKeyframe = true;
                    break;
                }
            }

            if (frameHasKeyframe)
            {
                // we only need to reconnect the disconnecting point by using the projected position from the disconnection
                FrameWorldPosition computedPosition = HistoricalPose::ComputeOffsetPosition({keyframePointer->WorldPosition,offsetToRemove});

                historicalPose.DisconnectFromKeyframe(keyframeId);

                // add in new connection to replace the one we removed
                for (const Id<Keyframe>& similarKeyframeId : similarKeyframes)
                {
                    // ensure that we dont already have this keyframe ID so that we don't create a double entry
                    // ensure that we don't accidentally link to ourselves
                    if (std::find(historicalPose.GetKeyframeOffsets().begin(),
                        historicalPose.GetKeyframeOffsets().end(), similarKeyframeId) == historicalPose.GetKeyframeOffsets().end()
                        && similarKeyframeId != historicalPose.GetId())
                    {
                        auto linkPointer = m_keyframes.find(similarKeyframeId);
                        assert(linkPointer != m_keyframes.end());

                        historicalPose.ConnectToKeyframe(HistoricalPose::ComputeFrameOffset(computedPosition, linkPointer->WorldPosition, similarKeyframeId));

                        break;  // exit the while loop, we have found a replacement
                    }
                }
            }
        }

        m_keyframes.erase(keyframePointer);

        // assert that the keyframe is not in the list
        assert(m_keyframes.find(keyframeId) == m_keyframes.end());
    }

    // ConnectAdjustedKeyframeToNewlyEstimatedPoses connects newly created poses to this keyframe.
    // This is used to make sure that a frame is connected to some keyframe after the one it was created with to help interpolation towards future keyframe.
    void PoseHistory::ConnectAdjustedKeyframeToNewlyEstimatedPoses(const Id<Keyframe>& keyframeId, const Pose& pose)
    {
        // assert that the keyframe is in the list
        assert(m_keyframes.find(keyframeId) != m_keyframes.end());

        // read backwards so that we we only insert keyframe on to the newest points.
        for (auto historicalPose = m_historicalPoses.rbegin(); historicalPose != m_historicalPoses.rend(); ++historicalPose)
        {
            if (historicalPose->GetId() < keyframeId)
            {
                if (historicalPose->GetKeyframeOffsets().size() < m_poseHistorySettings.MaxInterpolationConnections)
                {
                    auto offset = HistoricalPose::ComputeFrameOffset(ComputePoseForFrame(*historicalPose), ToWorldPosition(pose), keyframeId);
                    historicalPose->ConnectToKeyframe(offset);
                }
                else
                {
                    // If we have found one that already has Max then everything before this should of been processed. So we can stop here.
                    break;
                }
            }
        }
    }

    void PoseHistory::KeyframeAdded(const Id<Keyframe>& keyframeId, const Pose& pose)
    {
        // assert that the keyframe is not in the list
        assert(m_keyframes.find(keyframeId) == m_keyframes.end());

        m_keyframes.insert_presorted(PoseHistoryKeyframe{ keyframeId, ToWorldPosition(pose) });

        // assert that the keyframe is in the list
        assert(m_keyframes.find(keyframeId) != m_keyframes.end());
    }

    void PoseHistory::UpdateKeyframePose(const Id<Keyframe>& keyframeId, const Pose& keyframePose)
    {
        auto keyframePointer = m_keyframes.find(keyframeId);
        assert(keyframePointer != m_keyframes.end());

        keyframePointer._Ptr->WorldPosition = ToWorldPosition(keyframePose);
    }

    boost::optional<PoseHistory::TrackingInformation> PoseHistory::GetTrackingInformationForFrame(const FrameId& frameId) const
    {
        const auto targetElementPointer = m_historicalPoses.find(frameId);

        // Querying unknown frame ids is currently unsupported.
        if (targetElementPointer == m_historicalPoses.end())
        {
            assert("Unsupported usage: should never query historical poses that didn't track!");
            return boost::none;
        }
        // TODO support interpolating poses for times that we don't actually have by lerping between the two nearest poses

        // special case to handle that the tracking frame IS actually a keyframe
        auto keyframePointer = m_keyframes.find(targetElementPointer->GetId());
        if (keyframePointer != m_keyframes.end())
        {           
           return PoseHistory::TrackingInformation{ frameId, ToPose(keyframePointer->WorldPosition), targetElementPointer->GetDistortedCalibration().CreateCameraModel(), targetElementPointer->GetDepth() };  
        }
        else
        {
            // now we can compute the interpolated position and return
            return PoseHistory::TrackingInformation{ frameId, ToPose(ComputePoseForFrame(*targetElementPointer)), targetElementPointer->GetDistortedCalibration().CreateCameraModel(), targetElementPointer->GetDepth() };
        }
    }

    FrameWorldPosition PoseHistory::ComputePoseForFrame(const HistoricalPose& historicalPose) const
    {
        const HistoricalPoseOffsetVector& offsets = historicalPose.GetKeyframeOffsets();
        //TODO we can use stack memory here instead of heap
        std::vector<FramePositionRefPair> pairs;
        pairs.reserve(offsets.size());

        // we need to gather the pose information for the connected keyframes of the target pose and 
        // build up FramePositionPairs to compute an interpolated pose
        for (const HistoricalPoseOffset& offset : offsets)
        {
            // lookup the keyframe position for the target
            const auto historyElement = m_keyframes.find(offset.KeyframeId);
            assert(historyElement != m_keyframes.end());

            pairs.emplace_back(historyElement->WorldPosition, offset);
        }

        // now we can compute the interpolated position and return
        return HistoricalPose::ComputeWorldPosition(pairs);
    }

    PoseHistory::historical_pose_vector::iterator PoseHistory::FindByKeyframeId(const Id<Keyframe>& keyframeId)
    {
        // find the keyframe with the ID
        // TODO this is dumb, instead keyframes could store their time and we could modify the interfaces to reflect this behavior
        // TODO alternatively, we could binary search because the IDs are in insertion order, should add an assert at isertion time to support this optimization
        // TODO this probably doesn't matter once we switch the keyframe IDs to be the id passed in with the image

        return std::find_if(m_historicalPoses.begin(), m_historicalPoses.end(),
            [keyframeId](const HistoricalPose& frame) { return keyframeId == frame.GetId(); });
    }


    void PoseHistory::DebugGetAllPoses(std::vector<Pose>& poses) const
    {
        poses.clear();
        poses.reserve(m_historicalPoses.size());

        std::transform(m_historicalPoses.begin(), m_historicalPoses.end(), back_inserter(poses),
            [this](const HistoricalPose& hist)
            {
                return GetTrackingInformationForFrame(hist.GetFrameId())->Pose;
            });
    }

}
