// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "PoseEstimator.h"
#include "MageSettings.h"
#include "Map/ThreadSafeMap.h"

#include "Reprojection.h"
#include "FeatureMatcher.h"

#include "Utils/cv.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Utils/Logging.h"
#include <arcana/iterators.h>
#include "Fuser/Fuser.h"

#include <memory>
#include <chrono>
#include <numeric>

#include <arcana/analysis/data_point.h>
#include <arcana/analysis/object_trace.h>

namespace mage
{
    void FindInterpolatedPoses(const Pose & olderPose,
        const Quaternion& newerHistoricalQuaternion,
        const cv::Vec3f& newerHistoricalTranslation,
        float percentage,
        Quaternion& fractionalDeltaRotation,
        cv::Vec3f& fractionalDeltaTranslation)
    {
        assert(percentage >= 0.0f);

        // collect info about historical pose
        auto olderHistoricalWorldMat = olderPose.GetInverseViewMatrix();
        Quaternion olderHistoricalQuaternion = ToQuat(Rotation(olderHistoricalWorldMat));
        cv::Vec3f olderHistoricalTranslation = Translation(olderHistoricalWorldMat);

        //find delta of translation
        cv::Vec3f historicalDeltaTranslation = newerHistoricalTranslation - olderHistoricalTranslation;

        //find delta of rotation
        // can use conjugate because the quaternion is normalized
        Quaternion historicalDeltaRotation = newerHistoricalQuaternion * olderHistoricalQuaternion.conjugate();

        //position first
        fractionalDeltaTranslation = percentage * historicalDeltaTranslation;

        //extrapolation portion (slerp only valid between 0.0 and 1.0)
        fractionalDeltaRotation = Quaternion::Identity();
        while (percentage >= 1.0f)
        {
            fractionalDeltaRotation = historicalDeltaRotation * fractionalDeltaRotation;
            percentage -= 1.0f;
        }
        percentage = std::max(percentage, 0.0f);

        //get fractional components (interpolation)
        if (percentage > 0.0f)
        {
            fractionalDeltaRotation = Quaternion::Identity().slerp(percentage, historicalDeltaRotation) * fractionalDeltaRotation;
        }
        fractionalDeltaRotation.normalize();
    }

    void FindInterpolatedPoses(const Pose & olderPose,
        const Pose & newerPose,
        const float percentage,
        Quaternion& fractionalDeltaRotation,
        cv::Vec3f& fractionalDeltaTranslation)
    {
        // collect info about historical poses
        auto newerHistoricalWorldMat = newerPose.GetInverseViewMatrix();
        Quaternion newerHistoricalQuaternion = ToQuat(Rotation(newerHistoricalWorldMat));
        cv::Vec3f newerHistoricalTranslation = Translation(newerHistoricalWorldMat);

        FindInterpolatedPoses(olderPose,
            newerHistoricalQuaternion,
            newerHistoricalTranslation,
            percentage,
            fractionalDeltaRotation,
            fractionalDeltaTranslation);
    }

    Pose EstimateNextPoseFromHistory(const TrackingFrameHistory& previousFrames, const std::chrono::system_clock::time_point& nextPoseTime)
    {
        //TODO: if we choose not to bundle adjust history we can extend the history further back and perform filtering or higher order extrapolation
        
        assert(previousFrames.size() > 0);

        if (previousFrames.size() == 1)
            return previousFrames.newest().UpdatedPose;

        const Pose& newerPose = previousFrames.newest().UpdatedPose;
        auto newerHistoricalWorldMat = newerPose.GetInverseViewMatrix();
        Quaternion newerHistoricalQuaternion = ToQuat(Rotation(newerHistoricalWorldMat));
        cv::Vec3f newerHistoricalTranslation = Translation(newerHistoricalWorldMat);
        const std::chrono::system_clock::time_point& newerTimestamp = previousFrames.newest().Keyframe->GetAnalyzedImage()->GetTimeStamp();

        const Pose& olderPose = previousFrames.oldest().UpdatedPose;
        const std::chrono::system_clock::time_point& olderTimestamp = previousFrames.oldest().Keyframe->GetAnalyzedImage()->GetTimeStamp();

        assert(newerTimestamp > olderTimestamp && "Historical queue is out of order");

        //find delta of time            
        const std::chrono::duration<float, std::ratio<1, 1>> historicalDeltaSeconds = newerTimestamp - olderTimestamp;

        //find ratio with current frame to history
        assert(nextPoseTime > newerTimestamp && "The time predicting for is <= the most recent timestamp in the historical queue");
        const std::chrono::duration<float, std::ratio<1, 1>> currentDeltaSeconds = nextPoseTime - newerTimestamp;
        float timeRatio = currentDeltaSeconds.count() / historicalDeltaSeconds.count();

        //find interpolated rotation and translation
        Quaternion fractionalDeltaRotation;
        cv::Vec3f fractionalDeltaTranslation;

        FindInterpolatedPoses(olderPose,
            newerHistoricalQuaternion, newerHistoricalTranslation,
            timeRatio, fractionalDeltaRotation, fractionalDeltaTranslation);

        //generate predicted pose
        //TODO: normalize required?
        Quaternion predictedQuaternion = fractionalDeltaRotation * newerHistoricalQuaternion;
        predictedQuaternion.normalize();

        cv::Vec3f predictedTranslation = newerHistoricalTranslation + fractionalDeltaTranslation;

        return Pose(To4x4(ToMat(predictedQuaternion), predictedTranslation));
    }


    PoseEstimator::PoseEstimator(
        const PoseEstimationSettings& poseSettings,
        const RelocalizationSettings& relocSettings)
        :   m_poseSettings{ poseSettings },
            m_relocSettings{ relocSettings }
    {}

    bool PoseEstimator::TryEstimatePoseWithPrior(
        const TrackingFrameHistory& previousFrames,
        const Pose& prior,
        unsigned int minimumFeatureMatches,
        thread_memory memory,
        KeyframeEstimate::MutableViewT keyframe)
    {
        SCOPE_TIMER(PoseEstimator::TryEstimatePoseWithPrior);

        keyframe->SetPose(prior);

        auto history = memory.stack_vector<const KeyframeProxy*>();
        std::transform(previousFrames.begin(), previousFrames.end(), back_inserter(history), [](auto& kf) { return kf.Keyframe.get(); });

        return TryEstimatePoseFromKeyframe(history, minimumFeatureMatches, memory, keyframe);
    }

    namespace
    {
        struct BundleResult
        {
            Pose AdjustedPose;
            size_t Outliers;
        };

        BundleResult BundleAdjustPose(
            const std::vector<MapPointAssociations<MapPointTrackingProxy>::Association>& mapPoints,
            const Pose& pose, const AnalyzedImage& image,
            const KeyframeProxy& candidate,
            const RelocalizationSettings& relocSettings,
            mira::determinator& determinator,
            bool computeOutliers,
            thread_memory& memory)
        {
            Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints> keyframe{ Id<Keyframe>{}, candidate.GetAnalyzedImage(), pose, candidate.GetUndistortedIntrinsics(), candidate };

            AdjustableData bundled;
            bundled.Keyframes.emplace_back(keyframe);

            for (auto& association : mapPoints)
            {
                bundled.MapPoints.emplace_back(association.MapPoint);

                bundled.MapPointAssociations.push_back({
                    image.GetKeyPoint(association.Index).pt,
                    association.MapPoint.GetId(),
                    keyframe.GetId()
                });
            }

            BundleAdjust ba{ bundled, determinator, memory };

            BundleAdjustSettings settings;
            const unsigned int BASteps = relocSettings.BundleAdjustIterations;
            settings.NumSteps = BASteps;
            settings.NumStepsPerRun = BASteps;
            settings.MinSteps = BASteps;
            settings.MaxOutlierError = static_cast<float>(relocSettings.MaxBundleAdjustReprojectionError);
            ba.RunBundleAdjustment(BundleAdjust::NoOpScheduler{}, settings, /* fixMapPoints */ true, memory);

            return BundleResult{
                ba.GetData().Keyframes[0].GetPose(),
                computeOutliers ? ba.GetOutliers().size() : 0
            };
        };

        void RemoveMapPointsThatAreBehind(const Pose& pose, std::vector<MapPointAssociations<MapPointTrackingProxy>::Association>& mapPoints)
        {
            auto newEnd = std::remove_if(mapPoints.begin(), mapPoints.end(), [&](const MapPointAssociations<MapPointTrackingProxy>::Association& assoc)
            {
                return (assoc.MapPoint.GetPosition() - pose.GetWorldSpacePosition()).dot(pose.GetWorldSpaceForward()) <= 0;
            });
            mapPoints.erase(newEnd, mapPoints.end());
        };
    }

    boost::optional<size_t> PoseEstimator::TryEstimatePoseFromCandidates(
        BaseBow& bagOfWords,
        const collection<KeyframeProxy>& candidates,
        mira::determinator& determinator,
        thread_memory memory,
        KeyframeEstimate::MutableViewT keyframe)
    {
        SCOPE_TIMER(PoseEstimator::TryEstimatePoseFromCandidates);

        const auto& currentFrame = keyframe->GetAnalyzedImage();

        // if the number of features in the query image is less than the number of inliers that will be needed 
        // later don't even bother, this frame can't reloc.
        unsigned int currentFrameKeypointCount = gsl::narrow_cast<unsigned int>(currentFrame->GetDescriptorsCount());
        if (currentFrameKeypointCount < m_relocSettings.MinBruteForceCorrespondences)
            return{};

        boost::optional<size_t> indexOfSuccess{};
        std::vector<bool> currentFrameKeypointMask(currentFrameKeypointCount, true);

        struct RelocData
        {
            bool worthwhile = true;
            Pose pose;
            temp::vector<cv::DMatch> matches;
            std::vector<cv::Point2f> matchedKeyPoints;
            std::vector<cv::Point3f> matchedMapPoints;

            RelocData(thread_memory& memory)
                : matches{ memory.stack_vector<cv::DMatch>() }
            {}
        };

        auto relocdata = memory.stack_vector<RelocData>();
        relocdata.reserve(candidates.size());
        
        for (size_t i = 0; i < candidates.size(); ++i)
            relocdata.emplace_back(memory);

        auto tempKeyframeMatcher = bagOfWords.CreateFeatureMatcher({}, currentFrame->GetDescriptors());

        // round robin the candidates in order to find the one that positions the new image the "best"
        for(size_t iteration = 0; iteration < m_relocSettings.RoundRobinIterations && !indexOfSuccess; ++iteration)
        for (size_t idxCurrentCandidate = 0; idxCurrentCandidate < candidates.size() && !indexOfSuccess; idxCurrentCandidate++)
        {
            RelocData& data = relocdata[idxCurrentCandidate];
            if (!data.worthwhile)
                continue;

            const KeyframeProxy& keyframeCandidate = candidates[idxCurrentCandidate];
            if (keyframeCandidate.GetAssociatedKeypointCount() < m_relocSettings.MinBruteForceCorrespondences)
            {
                data.worthwhile = false;
                continue;
            }

            // initialize ransac data
            if (iteration == 0)
            {
                data.matches = memory.stack_vector<cv::DMatch>(keyframeCandidate.GetAssociatedKeypointCount());
                unsigned int numMatches = IndexedMatch(
                    bagOfWords,
                    nullptr, tempKeyframeMatcher.get(),
                    keyframeCandidate.GetId(), {},
                    keyframeCandidate.GetAnalyzedImage(), currentFrame,
                    keyframeCandidate.GetAssociatedKeypointMask(), currentFrameKeypointMask,
                    keyframeCandidate.GetAssociatedKeypointCount(), currentFrameKeypointCount,
                    m_relocSettings.OrbMatcherSettings.MaxHammingDistance,
                    m_relocSettings.OrbMatcherSettings.MinHammingDifference,
                    memory,
                    data.matches);

                if (numMatches < m_relocSettings.MinBruteForceCorrespondences)
                {
                    data.worthwhile = false;
                    continue;
                }

                // collect 2d and 3d points that were matched
                data.matchedKeyPoints.reserve(data.matches.size());
                data.matchedMapPoints.reserve(data.matches.size());

                for (const auto& match : data.matches)
                {
                    const MapPointTrackingProxy* pMapPointProxy = keyframeCandidate.GetAssociatedMapPoint(KeypointDescriptorIndex(match.queryIdx));
                    assert(pMapPointProxy && "we pass in a mask to the matcher, we should only be getting matches that are associated");

                    data.matchedMapPoints.emplace_back(pMapPointProxy->GetPosition());
                    data.matchedKeyPoints.emplace_back(currentFrame->GetKeyPoint(match.trainIdx).pt);
                }
            }

            // perform ransac iterations on each candidate to find a pose estimate
            std::vector<int> inliers;
            {
                SCOPE_TIMER(PoseEstimator::PnPSingleIteration);

                // use the previous estimations pose as the starting point when we already have a guess
                bool usePoseEstimate = iteration > 0;

                if (!PNPRansac(currentFrame->GetUndistortedCalibration(), data.matchedMapPoints, data.matchedKeyPoints,
                    usePoseEstimate, m_relocSettings.MaxBundlePnPReprojectionError, m_relocSettings.RansacConfidence, m_relocSettings.RansacIterations, data.pose, inliers))
                {
                    data.worthwhile = false;
                    continue;
                }
            }

            // Adjust the Pose estimate found by PnPRansac if it has enough inliers
            {
                std::vector<MapPointAssociations<MapPointTrackingProxy>::Association> mapPoints;
                mapPoints.reserve(inliers.size());
                for (size_t inlier : inliers)
                {
                    auto mapPoint = keyframeCandidate.GetAssociatedMapPoint(gsl::narrow<KeypointDescriptorIndex>(data.matches[inlier].queryIdx));
                    assert(mapPoint && "we matched on this index, it should never be null");
                    mapPoints.push_back({ *mapPoint, gsl::narrow<KeypointDescriptorIndex>(data.matches[inlier].trainIdx) });
                }

                RemoveMapPointsThatAreBehind(data.pose, mapPoints);
                if (mapPoints.size() / (float)data.matchedKeyPoints.size() < m_relocSettings.RansacInliersPctRequired)
                {
                    continue;
                }

                SCOPE_TIMER(PoseEstimator::OptimizePnpPose);

                auto result = BundleAdjustPose(mapPoints, data.pose, *currentFrame, keyframeCandidate, m_relocSettings, determinator, /*computeOutliers*/false, memory);
                data.pose = result.AdjustedPose;
            }

            // perform a guided search of more matches using the candidate's map points
            std::vector<KeypointDescriptorIndex> referenceIndex;
            std::vector<cv::KeyPoint> referenceKeyPoints;
            std::vector<ORBDescriptor::Ref> referenceDescriptors;
            std::vector<cv::Point3f> points3d;
            std::vector<cv::Point2f> predictedPositions;

            const cv::Matx34f viewMatrix = data.pose.GetViewMatrix();
            const cv::Matx33f cameraMatrix = currentFrame->GetUndistortedCalibration().GetCameraMatrix();

            keyframeCandidate.IterateAssociations([&](const MapPointTrackingProxy& mp, KeypointDescriptorIndex idx)
            {
                const Projection projection = ProjectUndistorted(viewMatrix, cameraMatrix, mp.GetPosition());
                if (projection.Distance >= 0)
                {
                    predictedPositions.emplace_back(projection.Point);
                    referenceIndex.push_back(idx);
                    referenceKeyPoints.push_back(keyframeCandidate.GetAnalyzedImage()->GetKeyPoint(idx));
                    referenceDescriptors.emplace_back(keyframeCandidate.GetAnalyzedImage()->GetDescriptor(idx));
                    points3d.push_back(mp.GetPosition());
                }
            });

            std::vector<cv::DMatch> good_matches;

            // search for matches in predicted location of keypoints
            auto matchCount = RadiusMatch(
                referenceKeyPoints,
                &predictedPositions,
                nullptr,
                referenceDescriptors,
                currentFrame->GetKeyPoints(),
                currentFrame->GetKeypointSpatialIndex(),
                nullptr,
                currentFrame->GetDescriptors(),
                m_relocSettings.SearchRadius,
                m_poseSettings.OrbMatcherSettings.MaxHammingDistance,
                m_poseSettings.OrbMatcherSettings.MinHammingDifference,
                memory,
                good_matches);

            if (matchCount < m_relocSettings.MinRadiusMatchCorrespondences)
            {
                continue;
            }

            std::vector<MapPointAssociations<MapPointTrackingProxy>::Association> mapPoints;
            mapPoints.reserve(good_matches.size());
            for (const auto& match : good_matches)
            {
                auto mapPoint = keyframeCandidate.GetAssociatedMapPoint(referenceIndex[match.queryIdx]);
                assert(mapPoint && "we matched on this index, it should never be null");
                mapPoints.push_back({ *mapPoint, gsl::narrow<KeypointDescriptorIndex>(match.trainIdx) });
            }

            RemoveMapPointsThatAreBehind(data.pose, mapPoints);
            if (mapPoints.size() < m_relocSettings.MinMapPoints)
            {
                continue;
            }

            DETERMINISTIC_CHECK(determinator, keyframeCandidate);
            DETERMINISTIC_CHECK(determinator, data.pose);

            // optimize the pose again using the radius matched correspondences
            // and return as result if supported by enough inliers
            {
                SCOPE_TIMER(PoseEstimator::OptimizeFinalPose);

                auto result = BundleAdjustPose(mapPoints, data.pose, *currentFrame, keyframeCandidate, m_relocSettings, determinator, /*computeOutliers*/true, memory);

                size_t baInlierCount = mapPoints.size() - result.Outliers;
                if (baInlierCount / (float)mapPoints.size() < m_relocSettings.BundleAdjustInliersPctRequired)
                {
                    data.worthwhile = false;
                    continue;
                }

                keyframe->SetPose(result.AdjustedPose);
                for (auto& assoc : mapPoints)
                    keyframe->AddAssociation(assoc.MapPoint, assoc.Index);
                DETERMINISTIC_CHECK(determinator, *keyframe);
                indexOfSuccess = idxCurrentCandidate;
            }
        }

        return indexOfSuccess;
    }
    
    bool PoseEstimator::TryEstimatePoseFromKeyframe(
        gsl::span<const KeyframeProxy*> referenceFrames,
        unsigned int minimumFeatureMatches,
        thread_memory memory,
        KeyframeEstimate::MutableViewT keyframe)
    {
        //TODO: maybe lost needs to consider inlier percentage
        SCOPE_TIMER(PoseEstimator::TryEstimatePoseFromKeyframe);

        const auto& currentFrame = keyframe->GetAnalyzedImage();
        const Pose estimatedPose = keyframe->GetPose();

        size_t referencePointCount = std::accumulate(referenceFrames.begin(), referenceFrames.end(), (size_t)0,
            [](size_t sum, const auto& frame) { return sum + frame->GetMapPointCount(); });

        if (referencePointCount == 0)
        {
            return false;
        }

        auto referenceMapPoints = memory.stack_vector<MapPointTrackingProxy::ViewT>(referencePointCount);
        auto referenceKeypoints = memory.stack_vector<cv::KeyPoint>(referencePointCount);
        auto referenceDescriptors = memory.stack_vector<ORBDescriptor::Ref>(referencePointCount);

        auto referenceMapPointsSet = memory.stack_unique_vector<Id<MapPoint>>(referencePointCount);

        const cv::Matx34f& viewMatrix = estimatedPose.GetViewMatrix();
        const cv::Matx33f& cameraMatrix = currentFrame->GetUndistortedCalibration().GetCameraMatrix();

        std::vector<cv::Point2f> predictedPositions;

        for (const auto& referenceFrame : referenceFrames)
        {
            referenceFrame->IterateAssociations([&](const MapPointTrackingProxy& proxy, KeypointDescriptorIndex idx)
            {
                // only consider points which have been asjusted at least once
                if (proxy.GetRefinementCount() < m_poseSettings.MinMapPointRefinementCount)
                {
                    return;
                }

                const auto& id = proxy.GetId();
                auto itr = referenceMapPointsSet.find(id);
                if (itr == referenceMapPointsSet.end())
                {
                    referenceMapPointsSet.insert(proxy.GetId());

                    const Projection projection = ProjectUndistorted(viewMatrix, cameraMatrix, proxy.GetPosition());
                    if (projection.Distance >= 0)
                    {
                        predictedPositions.emplace_back(projection.Point);
                        referenceMapPoints.emplace_back(proxy);
                        const auto& analyzedImage = referenceFrame->GetAnalyzedImage();
                        referenceKeypoints.emplace_back(analyzedImage->GetKeyPoint(idx));
                        referenceDescriptors.emplace_back(analyzedImage->GetDescriptor(idx));
                    }
                }
            });
        }

        std::vector<cv::DMatch> good_matches;

        // search for matches in predicted location of keypoints
        auto matchCount = RadiusMatch(
            referenceKeypoints,
            &predictedPositions,
            nullptr,
            referenceDescriptors,
            currentFrame->GetKeyPoints(),
            currentFrame->GetKeypointSpatialIndex(),
            nullptr,
            currentFrame->GetDescriptors(),
            m_poseSettings.SearchRadius,
            m_poseSettings.OrbMatcherSettings.MaxHammingDistance,
            m_poseSettings.OrbMatcherSettings.MinHammingDifference,
            memory,
            good_matches);

        float accuracy = matchCount / (float) referenceKeypoints.size();

        FIRE_OBJECT_TRACE("PoseEstimate.NumRefKeypoints", this, (mira::make_data_point<float>(
            currentFrame->GetTimeStamp(),
            (float)referenceKeypoints.size())));

        FIRE_OBJECT_TRACE("PoseEstimate.MatchesSmallRadius", this, (mira::make_data_point<float>(
            currentFrame->GetTimeStamp(),
            (float)matchCount)));

        if (matchCount < minimumFeatureMatches || accuracy < m_poseSettings.FeatureSmallMatchRatioThreshold)
        {
            good_matches.clear();

            // we don't have enough matches, so try and look for
            // points in their previous frames position with a wider
            // search radius
            matchCount = RadiusMatch(
                referenceKeypoints,
                &predictedPositions,
                nullptr,
                referenceDescriptors,
                currentFrame->GetKeyPoints(),
                currentFrame->GetKeypointSpatialIndex(),
                nullptr,
                currentFrame->GetDescriptors(),
                m_poseSettings.WiderSearchRadius,
                m_poseSettings.OrbMatcherSettings.MaxHammingDistance,
                m_poseSettings.OrbMatcherSettings.MinHammingDifference,
                memory,
                good_matches);

            FIRE_OBJECT_TRACE("PoseEstimate.MatchesMediumRadius", this, (mira::make_data_point<float>(
                currentFrame->GetTimeStamp(),
                (float)matchCount)));
        }
        else
        {
            FIRE_OBJECT_TRACE("PoseEstimate.MatchesMediumRadius", this, (mira::make_data_point<float>(
                currentFrame->GetTimeStamp(),
                0.0f)));
        }

        if (matchCount < minimumFeatureMatches || matchCount / (float)referenceKeypoints.size() < m_poseSettings.FeatureSmallMatchRatioThreshold)
        {
            good_matches.clear();

            // we don't have enough matches, so try and look for
            // points in their previous frames position with a wider
            // search radius
            matchCount = RadiusMatch(
                referenceKeypoints,
                nullptr,
                nullptr,
                referenceDescriptors,
                currentFrame->GetKeyPoints(),
                currentFrame->GetKeypointSpatialIndex(),
                nullptr,
                currentFrame->GetDescriptors(),
                m_poseSettings.ExtraWiderSearchRadius,
                m_poseSettings.OrbMatcherSettings.MaxHammingDistance,
                m_poseSettings.OrbMatcherSettings.MinHammingDifference,
                memory,
                good_matches);

            FIRE_OBJECT_TRACE("PoseEstimate.MatchesWideRadius", this, (mira::make_data_point<float>(
                currentFrame->GetTimeStamp(),
                (float)matchCount)));
        }
        else
        {
            FIRE_OBJECT_TRACE("PoseEstimate.MatchesWideRadius", this, (mira::make_data_point<float>(
                currentFrame->GetTimeStamp(),
                0.0f)));
        }

        if (matchCount < minimumFeatureMatches)
        {
            return false;
        }

        // create the keyframe from the matched mapPoints
        keyframe->SetPose(estimatedPose);
        for (const auto& match : good_matches)
        {
            const auto& mapPoint = referenceMapPoints[match.queryIdx];
            keyframe->AddAssociation(*mapPoint, (size_t)match.trainIdx);
        }

        return true;
    }

    //get an estimated camera pose from a set of 2d points and associated 3d points
    bool PoseEstimator::PNPRansac(
        const CameraCalibration& calibration,
        const std::vector<cv::Point3f>& points3d,
        const std::vector<cv::Point2f>& points2d,
        bool useExtrinsicGuess,
        float maxReprojectionError,
        float ransacConfidence,
        unsigned int numIterations,
        Pose& pose,
        std::vector<int>& inliers)
    {
        SCOPE_TIMER(PoseEstimator::PNPRansac);

        // solve  PnpRansac to find camera pose     
        cv::Matx31d translationVector = pose.GetViewSpacePosition();
        cv::Matx31d rotationVector = pose.GetViewSpaceRodriguesRotation();

        if (!solvePnPRansac(
            points3d, points2d,
            calibration.GetCameraMatrix(), calibration.GetDistortionType() != calibration::DistortionType::None ? calibration.GetCVDistortionCoeffs() : cv::noArray(),
            rotationVector, translationVector, useExtrinsicGuess, numIterations, maxReprojectionError, ransacConfidence, inliers, cv::SOLVEPNP_ITERATIVE))
        {
            return false;
        }

        cv::Matx31f rodriguesF = { (float)rotationVector(0), (float)rotationVector(1), (float)rotationVector(2) };
        pose.SetViewSpacePositionAndRotation(translationVector, RotationFromRodrigues(rodriguesF));
        return true;
    }
}
