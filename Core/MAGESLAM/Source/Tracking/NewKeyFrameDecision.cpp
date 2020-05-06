// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MageSettings.h"

#include "NewKeyFrameDecision.h"
#include "Utils/Logging.h"

#include "Analysis/DataPoints.h"
#include <arcana/analysis/object_trace.h>

namespace mage
{
    namespace
    {
        void ComputeFrameStatistics(
            gsl::span<std::reference_wrapper<const KeyframeReprojection>> connectedKeyframes,
            const collection<Proxy<MapPoint>>& mapPoints,
            unsigned int& totalPointsInKRef)
        {
            // TODO:  rename MapPointAssociations::GetMapPoints to GetSortedMapPoints, and move this assert there.
            // Then, CR all code that calls it to see if there are any other places we can exploit this sorted property.
            assert(std::is_sorted(mapPoints.begin(), mapPoints.end()));

            unsigned int bestSharedMapPointsCount = 0;
            totalPointsInKRef = 0;
            for (const KeyframeReprojection& keyframeReprojections : connectedKeyframes)
            {
                unsigned int countPoints = set_intersection_count(
                    keyframeReprojections.Points.begin(), keyframeReprojections.Points.end(),
                    mapPoints.begin(), mapPoints.end());

                if (countPoints > bestSharedMapPointsCount)
                {
                    bestSharedMapPointsCount = countPoints;
                    totalPointsInKRef = gsl::narrow_cast<unsigned int>(keyframeReprojections.Points.size());
                }
            }
        }

        bool MovedEnough(gsl::span<std::reference_wrapper<const KeyframeReprojection>> connectedKeyframes,
            const Pose& pose,
            const float minPointDistance,
            const float MinFrameMoveToMinDepthRatio)
        {
            SCOPE_TIMER(NewKeyFrameDecision::MovedEnough);
            const float minDistance = minPointDistance * MinFrameMoveToMinDepthRatio;
            const float minDistanceSq = minDistance*minDistance;

            const auto worldPos = pose.GetWorldSpacePosition();
            for (const auto& kf : connectedKeyframes)
            {
                auto diff = kf.get().Pose.GetWorldSpacePosition() - worldPos;
                auto distanceSq = diff.dot(diff);

                if (distanceSq < minDistanceSq)
                {
                    return false;
                }
            }

            return true;
        }

        // generates a grid of the connected keypoints and assess their distribution and completeness.
        // returns true of the frame is deemed "connected enough"
        bool DenslyConnected(const KeyframeBuilder& keyframe, const MageSlamSettings& settings)
        {
            SCOPE_TIMER(NewKeyFrameDecision::DenslyConnected);

            const auto& cameraSettings = GetSettingsForCamera(settings, keyframe.GetAnalyzedImage()->GetCameraIdentity());
            const size_t gridWidth = cameraSettings.KeyframeDecisionGridWidth;
            const size_t gridHeight = cameraSettings.KeyframeDecisionGridHeight;

            // make a grid full of 0 counts
            cv::Mat counts{ (int)gridWidth, (int)gridHeight, CV_16U, cv::Scalar::all(0) };

            float gridWidthPixels = (float)gridWidth / (float)keyframe.GetAnalyzedImage()->GetWidth();
            float gridHeightPixels = (float)gridHeight / (float)keyframe.GetAnalyzedImage()->GetHeight();

            // walk the keypoints and fill up our grid
            const auto keyPointMask = keyframe.GetAssociatedKeypointMask();
            for (size_t i = 0; i < keyPointMask.size(); i++)
            {
                if (keyPointMask[i])
                {
                    const auto& keyPoint = keyframe.GetAnalyzedImage()->GetKeyPoint(i);
                    // compute the destination grid, bounding to the grid size to handle points which end up undistorted off image
                    size_t gridX = std::min(gridWidth-1, static_cast<size_t>(std::max(0.f, floor(keyPoint.pt.x * gridWidthPixels))));
                    size_t gridY = std::min(gridHeight-1, static_cast<size_t>(std::max(0.f, floor(keyPoint.pt.y * gridHeightPixels))));

                    counts.at<unsigned short>((int)gridX, (int)gridY)++;
                }
            }

            unsigned int emptyCount = 0;

            // count up the cells and compute the statistics for decision
            for (unsigned int i = 0; i < gridWidth; i++)
            {
                for (unsigned int j = 0; j < gridHeight; j++)
                {
                    if (counts.at<unsigned short>(i, j) < cameraSettings.KeyframeDecisionMinMapPointsPerGridCell)
                    {
                        emptyCount++;
                    }
                }
            }

            float emptyPercentage = (float)emptyCount / ((float)gridWidth * (float)gridHeight);

            FIRE_OBJECT_TRACE("NewKeyFrameDecision.EmptyPercentage", nullptr, make_frame_data_point(keyframe, 100*emptyPercentage));

            return emptyPercentage <= cameraSettings.KeyframeDecisionAllowedEmptyCellPercentage;
        }
    }

    NewKeyFrameDecision::NewKeyFrameDecision(const MageSlamSettings& settings)
        : m_frameCountSinceLastKeyFrame(0),
        m_frameCountSinceLastRelocalization(m_settings.KeyframeSettings.KeyframeDecisionMinFrameCount),
        m_settings{ settings }
    {}
    // returns true if this frame should become a keyframe
    //
    // The wording in Mur-Artal et al 2015 (section 5.E, point 4) is unclear about the requirement
    // for new keyframes that "current frame tracks less than 90% points than K_ref."  Two main
    // interpretations of this statement are as follows:
    //     (1) Current keyframe contains fewer than 90% of the pre-existing points in K_ref.
    //          this works on map init because when we only have two keyframes, we can only
    //          lose map points in common with KRef (track local map only creates associations, not map points)
    //     (2) Fewer than 90% of the points in the current keyframe are pre-existing in K_ref.
    // The difference between these interpretations can be seen in the case where
    //     K_ref = { A, B, C, D, E } where A, B, C, D, and E are pre-existing map points
    //     K_cur1 = { A, B, C, D }
    //     K_cur2 = { A, B, C, D, E, F } where F is is a new map point that can be seen in K_ref 
    //                                   and K_cur2, but did not pre-exist in K_ref
    // Interpretation 1 will accept K_cur1 and reject K_cur2, whereas interpretation 2 will
    // accept K_cur2 and reject K_cur1.
    //
    // TODO Interpretation 2 is currently favored and implemented.  Decide for the long run on 1, 2, or both.
    bool NewKeyFrameDecision::IsNewKeyFrame(
        bool mappingIsIdle,
        const KeyframeBuilder& keyframe,
        gsl::span<std::reference_wrapper<const KeyframeReprojection>> connectedKeyframes,
        const collection<Proxy<MapPoint>>& mapPoints,
        const bool relocalizationHappenedThisFrame,
        const float minPointDistance)
    {
        SCOPE_TIMER(NewKeyFrameDecision::IsNewKeyFrame);
        m_frameCountSinceLastKeyFrame++;
        FIRE_OBJECT_TRACE("NewKeyFrameDecision.FrameCountSinceLastKeyFrame", nullptr, make_frame_data_point(keyframe, m_frameCountSinceLastKeyFrame));

        if (relocalizationHappenedThisFrame)
        {
            m_frameCountSinceLastRelocalization = 0;
        }
        else
        {
            m_frameCountSinceLastRelocalization++;
        }
        FIRE_OBJECT_TRACE("NewKeyFrameDecision.FrameCountSinceLastRelocalization", nullptr, make_frame_data_point(keyframe, m_frameCountSinceLastRelocalization));

        if (!(m_frameCountSinceLastRelocalization > m_settings.KeyframeSettings.KeyframeDecisionMinFrameCount))
        {
            return false;
        }

        if (!(mappingIsIdle || m_frameCountSinceLastKeyFrame > m_settings.KeyframeSettings.KeyframeDecisionMinFrameCount))
        {
            return false;
        }

        FIRE_OBJECT_TRACE("NewKeyFrameDecision.MapPointCount", nullptr, make_frame_data_point(keyframe, keyframe.GetMapPointCount()));
        if (!(keyframe.GetMapPointCount() > m_settings.KeyframeSettings.KeyframeDecisionMinTrackingPointCount))
        {
            return false;
        }

        if (!(keyframe.GetMapPointCount() < m_settings.KeyframeSettings.KeyframeDecisionMaxTrackingPointMatches))
        {
            return false;
        }

        if (!MovedEnough(connectedKeyframes, keyframe.GetPose(), minPointDistance, m_settings.KeyframeSettings.MinFrameMoveToMinDepthRatio)
            && DenslyConnected(keyframe, m_settings))
        {
            return false;
        }

        unsigned int numMapPointsInKRef = 0;

        ComputeFrameStatistics(connectedKeyframes, mapPoints, numMapPointsInKRef);

        FIRE_OBJECT_TRACE("NewKeyFrameDecision.NumMapPointsInKRef", nullptr, make_frame_data_point(keyframe, numMapPointsInKRef));

        if (keyframe.GetMapPointCount() < numMapPointsInKRef * m_settings.KeyframeSettings.KeyframeDecisionMaxTrackingPointOverlap + m_settings.KeyframeSettings.KeyframeDecisionMinTrackingPointCount)
        {
            m_frameCountSinceLastKeyFrame = 0;
            return true;
        }

        return false;
    }
}
