// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once
#include <memory>

#include "KeyframeBuilder.h"
#include "Map\ThreadSafeMap.h"
#include "MageSettings.h"
#include "Tracking\KeyframeBuilder.h"
#include "Tracking\Reprojection.h"
#include "MageSettings.h"
#include "Device\device.h"

#include <array>

namespace UnitTests
{
    class TrackLocalMapUnitTest;
}

namespace mage
{
    class TrackLocalMap
    {
    public:

        struct ProjectedMapPoint
        {
            cv::Point3f MapPoint;
            cv::Point2f Projection;
            uint64_t MapPointRefinementCount;

            ProjectedMapPoint(const cv::Point3f& mp, const cv::Point2f& proj, const uint64_t& refinementCount)
                : MapPoint{ mp }, Projection{ proj }, MapPointRefinementCount{ refinementCount }
            {}
        };

        TrackLocalMap(const MageSlamSettings& settings, const device::IMUCharacterization& imuCharacterization, mira::determinator& determinator);
        ~TrackLocalMap();

        // Run TrackLocalMap and update the contentence of currentFrame
        bool RunTrackLocalMap(
            gsl::span<const KeyframeReprojection> connectedKeyframes,
            std::map<Id<MapPoint>, unsigned int> recentlyCreatedMapPoints,
            thread_memory memory,
            bool updateRecentMapPointScoring,
            bool unassociateOutliers,
            KeyframeProxy& currentFrame);
        
        static bool IsGoodCandidate(
            const std::shared_ptr<const AnalyzedImage>& image,
            const float imageBorder,
            const Projection& projectedPoint,
            const cv::Point3f& framePosition,
            const cv::Vec3f& frameForward,
            const Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics, proxy::ViewingData>::ViewT& point,
            float minDegreesBetweenCurrentViewAndMapPointView);

        static bool IsGoodCandidate(
            const std::shared_ptr<const AnalyzedImage>& image,
            const unsigned int imageBorder,
            const Pose& curFramePose,
            const Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics, proxy::ViewingData>::ViewT& point,
            float minDegreesBetweenCurrentViewAndMapPointView);

        static bool MatchMapPointToCurrentFrame(
            const std::shared_ptr<const AnalyzedImage>& image,
            const std::vector<bool>& unassociatedKeypointsMask,
            const cv::KeyPoint& mapPointKeypoint,
            const ORBDescriptor::Ref& mapPointDesc,
            const OrbMatcherSettings& orbMatcherSettings,
            const float matchSearchRadius,
            thread_memory memory,
            KeypointDescriptorIndex& keypointDescriptorIndex);

        static bool ProjectMapPointIntoCurrentFrame(const KeyframeProxy& currentFrame,
            const cv::Matx34f& viewMatrix,
            const cv::Matx33f& cameraCalibrationMatrix,
            const MapPointProxy& point,
            const cv::Point3f& framePosition,
            const cv::Vec3f& frameForward,
            float angle,
            float searchRadius,
            float imageBorder,
            float pyramidScale,
            size_t numLevels,
            const OrbMatcherSettings& orbMatcherSettings,
            const std::vector<bool>& unassociatedMask,
            thread_memory memory,
            bool& predicted,
            KeypointDescriptorIndex& keypointDescriptorIndex);

        static Pose OptimizeCameraPose(
            const Pose& currentFramePose,
            const CameraCalibration& cameraCalibration,
            gsl::span<const ProjectedMapPoint> mapPoints,
            unsigned int numIterations,
            float maxOutlierErrorSquared,
            float huberWidth,
            mira::determinator& determinator,
            thread_memory memory,
            std::vector<unsigned int>& outlierIndices);

        mira::unique_vector<Id<MapPoint>> GetRecentMapPointsThatFailScoring() const;

        static Pose GetUpdatedKeyframePoseRelativeToBasis(
            const Pose& keyframePose,
            const Pose& originalBasisPose,
            const Pose& updatedBasisPose);

    private:

        void PrepareForRecentMapPointScoring(
            std::map<Id<MapPoint>, unsigned int> recentlyCreatedMapPoints,
            thread_memory memory);

        struct MapPointScore
        {
            size_t Found;
            size_t Predicted;
        };

        std::map<Id<MapPoint>, unsigned int> m_recentlyCreatedMapPointIds;
        std::map<Id<MapPoint>, MapPointScore> m_recentlyCreatedMapPointsScoring;
        
        const TrackLocalMapSettings& m_trackLocalMapSettings;
        const OrbMatcherSettings& m_orbMatcherSettings;
        const FuserSettings& m_fuserSettings;
        const device::IMUCharacterization& m_imuCharacterization;
        mira::determinator& m_determinator;

        friend class ::UnitTests::TrackLocalMapUnitTest;
    };
}
