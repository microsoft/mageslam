// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

// Given a set of 3d to 2d correspondences, this class calculates
// the camera pose by solving a PnP [problem inside a RANSAC scheme.

#pragma once

#include "Data\Pose.h"

#include "KeyframeBuilder.h"
#include "utils\cv.h"
#include "utils\historical_queue.h"
#include <opencv2\core\core.hpp>
#include <memory>
#include <list>
#include <chrono>
#include <boost/optional.hpp>
 
namespace UnitTests
{
    class PoseEstimatorUnitTest;
}

namespace mira
{
    class determinator;
}

namespace mage
{
    class ThreadSafeMap;
    class BaseBow;
    class Fuser;

    struct HistoricalFrame
    {
        explicit HistoricalFrame(std::shared_ptr<KeyframeProxy> keyframe)
            :   UpdatedPose{ keyframe->GetPose() },
                Keyframe{ std::move(keyframe) }
        {}

        HistoricalFrame(const HistoricalFrame&) = default;
        HistoricalFrame(HistoricalFrame&&) = default;

        HistoricalFrame& operator =(const HistoricalFrame&) = default;
        HistoricalFrame& operator =(HistoricalFrame&&) = default;

        Pose UpdatedPose;
        std::shared_ptr<KeyframeProxy> Keyframe;
    };

    using TrackingFrameHistory = historical_queue<HistoricalFrame, 5>;

    using KeyframeEstimate = Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics, proxy::Associations<MapPointTrackingProxy>>;

    /*
    *   Estimate the pose of the next frame using the constant velocity motion model.
    *   The pair is the pose and the time of that pose
    *   'nextPoseTime' is the time to predict for the pose
    */
    Pose EstimateNextPoseFromHistory(const TrackingFrameHistory& previousFrames, const std::chrono::system_clock::time_point& nextPoseTime);

    void FindInterpolatedPoses(const Pose & olderPose,
        const Quaternion& newerHistoricalQuaternion,
        const cv::Vec3f& newerHistoricalTranslation,
        float percentage,
        Quaternion& fractionalDeltaRotation,
        cv::Vec3f& fractionalDeltaTranslation);

    void FindInterpolatedPoses(const Pose & olderPose,
        const Pose & newerPose,
        const float percentage,
        Quaternion& fractionalDeltaRotation,
        cv::Vec3f& fractionalDeltaTranslation);

    class PoseEstimator
    {
    public:
        PoseEstimator(const PoseEstimationSettings& settings, const RelocalizationSettings& relocSettings);

        bool TryEstimatePoseWithPrior(
            const TrackingFrameHistory& previousFrames,
            const Pose& prior,
            unsigned int minimumFeatureMatches,
            thread_memory memory,
            KeyframeEstimate::MutableViewT keyframe);

        boost::optional<size_t> TryEstimatePoseFromCandidates(
            BaseBow& ,
            const collection<KeyframeProxy>& candidates,
            mira::determinator& determinator,
            thread_memory memory,
            KeyframeEstimate::MutableViewT keyframe);

        bool TryEstimatePoseFromKeyframe(
            gsl::span<const KeyframeProxy*> referenceFrame,
            unsigned int minimumFeatureMatches,
            thread_memory memory,
            KeyframeEstimate::MutableViewT keyframe);

    private:
        static bool PNPRansac(
            const CameraCalibration& calibration,
            const std::vector<cv::Point3f>& Points3d,
            const std::vector<cv::Point2f>& Points2d,
            bool useExtrinsicGuess,
            float maxReprojectionError,
            float ransacConfidence,
            unsigned int numIterations,
            Pose& pose,
            std::vector<int>& inliers);

        const PoseEstimationSettings& m_poseSettings;
        const RelocalizationSettings& m_relocSettings;

        friend class ::UnitTests::PoseEstimatorUnitTest;
    };
}
