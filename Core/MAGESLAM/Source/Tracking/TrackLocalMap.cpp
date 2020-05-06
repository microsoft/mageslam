// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "TrackLocalMap.h"
#include "FeatureMatcher.h"
#include "Debugging/SkeletonLogger.h"
#include "Device/Device.h"
#include "Map/MappingMath.h"
#include "Tracking/Reprojection.h"
#include "Utils/cv.h"
#include "Utils/Logging.h"
#include <BundlerLib.h>
#include <arcana/iterators.h>
#include <arcana/math.h>
#include <algorithm>
#include <map>
#include <set>

#include "Analysis/DataPoints.h"
#include <arcana/analysis/object_trace.h>

using namespace std;

namespace mage
{
    TrackLocalMap::TrackLocalMap(const MageSlamSettings& settings, const device::IMUCharacterization& imuCharacterization, mira::determinator& determinator)
        : m_orbMatcherSettings{ settings.TrackLocalMapSettings.OrbMatcherSettings },
        m_trackLocalMapSettings{ settings.TrackLocalMapSettings },
        m_fuserSettings {settings.FuserSettings },
        m_imuCharacterization{ imuCharacterization },
        m_determinator{ determinator }
    {  }

    TrackLocalMap::~TrackLocalMap()
    {}

    bool TrackLocalMap::RunTrackLocalMap(
        gsl::span<const KeyframeReprojection> connectedKeyframes,
        std::map<Id<MapPoint>, unsigned int> recentlyCreatedMapPoints,
        thread_memory memory,
        bool updateRecentMapPointScoring,
        bool unassociateOutliers, 
        KeyframeProxy& currentFrame)
    {
        SCOPE_TIMER(TrackLocalMap::RunTrackLocalMap);

        if (connectedKeyframes.empty())
        {
            currentFrame.ClearAssociations();
            return false;
        }

        if (updateRecentMapPointScoring)
        {
            PrepareForRecentMapPointScoring(move(recentlyCreatedMapPoints), memory);
        }

        // because this is a set of map points from neighboring keyframes, we don't know ahead of time a good
        // upper bound for the number to expect.  This seems like a conservative upper bound to prevent reallocations
        const size_t initialMapPointSize = 500;

        auto mapPoints = memory.stack_vector<ProjectedMapPoint>(initialMapPointSize);
        auto mapPointIds = memory.stack_vector<Id<MapPoint>>(initialMapPointSize);

        SkeletonLogger::TrackLocalMap::LogPose(1, currentFrame, currentFrame.GetPose());
        SkeletonLogger::TrackLocalMap::LogAssociatedCount(1, currentFrame);

        currentFrame.IterateAssociations([&, this](const MapPointTrackingProxy& mapPoint, KeypointDescriptorIndex index)
            {
                const auto& keyPoint = currentFrame.GetAnalyzedImage()->GetKeyPoint(index);

                if (updateRecentMapPointScoring)
                {
                    auto mapPointItr = m_recentlyCreatedMapPointsScoring.find(mapPoint.GetId());
                    bool isRecentlyCreatedMapPoint = m_recentlyCreatedMapPointsScoring.end() != mapPointItr;
                    if (isRecentlyCreatedMapPoint)
                    {
                        MapPointScore& mapPointScoring = mapPointItr->second;
                        mapPointScoring.Found++;
                        mapPointScoring.Predicted++;
                    }
                }

                mapPoints.emplace_back(mapPoint.GetPosition(), keyPoint.pt, mapPoint.GetRefinementCount());
                mapPointIds.emplace_back(mapPoint.GetId());
            });

        std::vector<unsigned int> outliers;
        outliers.reserve(mapPoints.size());

        const float outlierErrorSquared = m_trackLocalMapSettings.MaxOutlierError * m_trackLocalMapSettings.MaxOutlierError;
        const float outlierErrorSquaredPoseEstimation = m_trackLocalMapSettings.MaxOutlierErrorPoseEstimation * m_trackLocalMapSettings.MaxOutlierErrorPoseEstimation;
        Pose updatedPose;
        {
            SCOPE_TIMER(TrackLocalMap::OptimizeCameraPose_1);
            updatedPose = OptimizeCameraPose(
                currentFrame.GetPose(),
                currentFrame.GetAnalyzedImage()->GetUndistortedCalibration(),
                mapPoints,
                m_trackLocalMapSettings.InitialPoseEstimateBundleAdjustmentSteps,
                outlierErrorSquaredPoseEstimation,
                m_trackLocalMapSettings.InitialPoseEstimateBundleAdjustmentHuberWidth,
                m_determinator,
                memory,
                outliers);
        }
        DETERMINISTIC_CHECK(m_determinator, updatedPose);
        DETERMINISTIC_CHECK(m_determinator, outliers.begin(), outliers.end());

        SkeletonLogger::TrackLocalMap::LogPose(2, currentFrame, updatedPose);
        SkeletonLogger::TrackLocalMap::LogAssociatedCount(2, currentFrame);
        FIRE_OBJECT_TRACE("TrackLocalMap.NumOutliers_1", nullptr, make_frame_data_point(currentFrame, (float)outliers.size()));

        std::map<Id<MapPoint>, KeypointDescriptorIndex> poseEstimationOutlierMatches;

        vector<bool> unassociatedMask = currentFrame.GetUnassociatedKeypointMask();
        {
            SCOPE_TIMER(TrackLocalMap::GatherMapPointsFromCovis);

            if (unassociateOutliers)
            {
                // sort the outliers in reverse order so that we can safely pull them out of the mapPoints vector
                std::sort(outliers.begin(), outliers.end(), std::greater<size_t>());

                for (const size_t index : outliers)
                {
                    const auto& mapPointId = mapPointIds[index];
                    const KeypointDescriptorIndex keypointIndex = currentFrame.GetAssociatedIndex(mapPointId);
                    poseEstimationOutlierMatches[mapPointId] = keypointIndex;
                    //Remove association, no need to update map after this remove because it has not been added to the map yet
                    unassociatedMask[keypointIndex] = true;
                    currentFrame.RemoveAssociation(mapPointId);
                    mapPoints.erase(mapPoints.begin()+index);
                    mapPointIds.erase(mapPointIds.begin() + index);

                    if (updateRecentMapPointScoring)
                    {
                        auto mapPointItr = m_recentlyCreatedMapPointsScoring.find(mapPointId);
                        bool isRecentlyCreatedMapPoint = m_recentlyCreatedMapPointsScoring.end() != mapPointItr;
                        if (isRecentlyCreatedMapPoint)
                        {
                            MapPointScore& mapPointScoring = mapPointItr->second;
                            mapPointScoring.Found--;
                        }
                    }
                }
            }

            if (mapPoints.empty())
                return false;

            cv::Point3f framePosition = updatedPose.GetWorldSpacePosition();
            cv::Vec3f frameForward = updatedPose.GetWorldSpaceForward();

            auto visitedMapPoints = memory.stack_unordered_set<Id<MapPoint>>();
            visitedMapPoints.reserve(gsl::narrow_cast<size_t>(ceil(currentFrame.GetAnalyzedImage()->GetMaxFeatures() * connectedKeyframes.size())));

            for (const auto& id : mapPointIds)
            {
                visitedMapPoints.insert(id);
            }

            FIRE_OBJECT_TRACE("TrackLocalMap.NumCovisKeyframes", nullptr, make_frame_data_point(currentFrame, (float)connectedKeyframes.size()));

            const cv::Matx33f& cameraCalibrationMatrix = currentFrame.GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix();
            const cv::Matx34f& viewMatrix = updatedPose.GetViewMatrix();

            //if we aren't connected to any keyframes, we should be lost
            if (connectedKeyframes.size() < 1)
            {
                currentFrame.ClearAssociations();
                return false;
            }

            for (const auto& keyframePoints : connectedKeyframes)
            {
                for (const auto& point : keyframePoints.Points)
                {
                    // ignore points which haven't been adjusted a single time
                    if (point.GetRefinementCount() < m_trackLocalMapSettings.MinMapPointRefinementCount)
                    {
                        continue;
                    }

                    if (visitedMapPoints.find(point.GetId()) != visitedMapPoints.end())
                    {
                        // if we already have this map point continue
                        continue;
                    }
                    visitedMapPoints.insert(point.GetId());

                    // don't allow a match for an already found outlier
                    const auto& mapPointEntry = poseEstimationOutlierMatches.find(point.GetId());
                    bool featureHidden = false;
                    if (mapPointEntry != poseEstimationOutlierMatches.end())
                    {
                        // sometimes the feature may have already been matched to a different map point, in this case no need to hide the feature or unhide it later
                        if (unassociatedMask[mapPointEntry->second])
                        {
                            unassociatedMask[mapPointEntry->second] = false;
                            featureHidden = true;
                        }
                    }

                    KeypointDescriptorIndex keypointDescriptorIndex = std::numeric_limits<KeypointDescriptorIndex>::max();
                    bool predicted = false;
                    bool found = ProjectMapPointIntoCurrentFrame(currentFrame,
                        viewMatrix,
                        cameraCalibrationMatrix,
                        point,
                        framePosition,
                        frameForward,
                        m_trackLocalMapSettings.MinDegreesBetweenCurrentViewAndMapPointView,
                        m_trackLocalMapSettings.MatchSearchRadius,
                        currentFrame.GetAnalyzedImage()->GetImageBorder(),
                        currentFrame.GetAnalyzedImage()->GetPyramidScale(),
                        currentFrame.GetAnalyzedImage()->GetNumLevels(),
                        m_orbMatcherSettings,
                        unassociatedMask,
                        memory,
                        predicted,
                        keypointDescriptorIndex);

                    // undo the mask override if it was done above
                    if (featureHidden)
                    {
                        assert(mapPointEntry != poseEstimationOutlierMatches.end() && "Only expect to hide features which were outlier matched");
                        unassociatedMask[mapPointEntry->second] = true;
                    }

                    // recent map point culling indicates it should keep track of a count of how often recent map points were found
                    // when they were predicted to be. interpreting prediction to mean, passes all of the tests above, not just on
                    // screen
                    if (predicted && updateRecentMapPointScoring)
                    {
                        auto mapPointItr = m_recentlyCreatedMapPointsScoring.find(point.GetId());
                        bool isRecentlyCreatedMapPoint = m_recentlyCreatedMapPointsScoring.end() != mapPointItr;
                        if (isRecentlyCreatedMapPoint)
                        {
                            MapPointScore& mapPointScoring = mapPointItr->second;
                            mapPointScoring.Predicted++;
                            if (found)
                            {
                                mapPointScoring.Found++;
                            }
                        }
                    }

                    if (found)
                    {
                        currentFrame.AddAssociation(point.As<MapPointTrackingProxy>(), keypointDescriptorIndex);
                        unassociatedMask[keypointDescriptorIndex] = false;
                        mapPoints.emplace_back(ProjectedMapPoint{point.GetPosition(),currentFrame.GetAnalyzedImage()->GetKeyPoint(keypointDescriptorIndex).pt, point.GetRefinementCount() });
                        mapPointIds.emplace_back(point.GetId());
                    }
                }
            }

            outliers.clear();
            outliers.reserve(mapPoints.size());

            FIRE_OBJECT_TRACE("TrackLocalMap.NumVisitedMapPoints", nullptr, make_frame_data_point(currentFrame, (float)visitedMapPoints.size()));

            SkeletonLogger::TrackLocalMap::LogVisitedMapPoints(visitedMapPoints);
        }

        FIRE_OBJECT_TRACE("TrackLocalMap.NumMatchedKeypoints", nullptr, make_frame_data_point(currentFrame,(float)mapPoints.size()));

        {
            SCOPE_TIMER(TrackLocalMap::OptimizeCameraPose_2);
            updatedPose = OptimizeCameraPose(
                updatedPose,
                currentFrame.GetAnalyzedImage()->GetUndistortedCalibration(),
                mapPoints,
                m_trackLocalMapSettings.BundleAdjustmentG2OSteps,
                outlierErrorSquared,
                m_trackLocalMapSettings.BundleAdjustmentHuberWidth,
                m_determinator,
                memory,
                outliers);
        }
        DETERMINISTIC_CHECK(m_determinator, updatedPose);
        DETERMINISTIC_CHECK(m_determinator, outliers.begin(), outliers.end());

        if (unassociateOutliers)
        {
            for (const size_t index : outliers)
            {
                const Id<MapPoint>& mapPointId = mapPointIds[index];
                //Remove association, no need to update map after this remove because it has not been added to the map yet
                unassociatedMask[currentFrame.GetAssociatedIndex(mapPointId)] = true;
                currentFrame.RemoveAssociation(mapPointId);

                if (updateRecentMapPointScoring)
                {
                    auto mapPointItr = m_recentlyCreatedMapPointsScoring.find(mapPointId);
                    bool isRecentlyCreatedMapPoint = m_recentlyCreatedMapPointsScoring.end() != mapPointItr;
                    if (isRecentlyCreatedMapPoint)
                    {
                        MapPointScore& mapPointScoring = mapPointItr->second;
                        mapPointScoring.Found--;  //pair<found, predicted>
                    }
                }
            }
        }

        FIRE_OBJECT_TRACE("TrackLocalMap.NumOutliers_2", nullptr, make_frame_data_point(currentFrame, (float)outliers.size()));

        // double-check that we have a valid pose
        if (currentFrame.GetAssociatedKeypointCount() < m_trackLocalMapSettings.MinTrackedFeatureCount)
        {
            currentFrame.ClearAssociations();
            return false;
        }

        currentFrame.SetPose(updatedPose);

        SkeletonLogger::TrackLocalMap::LogPose(3, currentFrame, currentFrame.GetPose());
        SkeletonLogger::TrackLocalMap::LogAssociatedCount(3, currentFrame);
        SkeletonLogger::TrackLocalMap::LogKeypoints(currentFrame);

        return true;
    }

    bool TrackLocalMap::ProjectMapPointIntoCurrentFrame(const KeyframeProxy& currentFrame,
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
        const vector<bool>& unassociatedMask,
        thread_memory memory,
        bool& predicted,
        KeypointDescriptorIndex& keypointDescriptorIndex)
    {
        predicted = false;

        Projection projected = ProjectUndistorted(viewMatrix, cameraCalibrationMatrix, point.GetPosition());

        if (!IsGoodCandidate(
            currentFrame.GetAnalyzedImage(),
            imageBorder,
            projected,
            framePosition,
            frameForward,
            point,
            angle))
        {
            return false;
        }

        cv::Point3f deltaPosition = point.GetPosition() - framePosition;
        float distanceSquare = deltaPosition.dot(deltaPosition);
        int octave = ComputeOctave(sqrtf(distanceSquare), point.GetDMin(), pyramidScale);
        // the predicted octave might be outside the possible range, if so this means that a match isn't possible,
        // no point in continuing to evaluate this map point
        // note that this shouldn't happen because the points should get thrown out by IsGoodCandidate
        // however, in practice this will occur due to floating point rounding on occasion
        if (octave < 0 || octave > static_cast<int>(numLevels))
        {
            return false;
        }

        predicted = true;

        // found a good candidate, now check for a match in the current frame
        // Find best decriptors to make new map point connections and add map point to current frame and update associated
        cv::KeyPoint mapPointKp(projected.Point, -1.0f, 0.0f, 0.0f, octave, -1);
        if (MatchMapPointToCurrentFrame(
            currentFrame.GetAnalyzedImage(),
            unassociatedMask,
            mapPointKp,
            point.GetRepresentativeDescriptor(),
            orbMatcherSettings,
            searchRadius,
            memory,
            keypointDescriptorIndex))
        {
            return true;
        }
        return false;
    }

    void TrackLocalMap::PrepareForRecentMapPointScoring(
        std::map<Id<MapPoint>, unsigned int> recentlyCreatedMapPoints,
        thread_memory memory)
    {
        SCOPE_TIMER(TrackLocalMap::PrepareForRecentMapPointScoring);

        // remove the map points that should no longer be tracked for successes
        auto noLongerTrackPoints = memory.stack_vector<pair<const Id<MapPoint>, unsigned int>>(max(m_recentlyCreatedMapPointIds.size(), recentlyCreatedMapPoints.size()));
        std::set_difference(m_recentlyCreatedMapPointIds.begin(), m_recentlyCreatedMapPointIds.end(),
            recentlyCreatedMapPoints.begin(), recentlyCreatedMapPoints.end(),
            mira::emplace_inserter(noLongerTrackPoints));

        for (const auto& curId : noLongerTrackPoints)
        {
            m_recentlyCreatedMapPointsScoring.erase(curId.first);
        }

        // add the map points we are now tracking for successes
        auto newTrackPoints = memory.stack_vector<pair<const Id<MapPoint>, unsigned int>>(max(m_recentlyCreatedMapPointIds.size(), recentlyCreatedMapPoints.size()));
        std::set_difference(recentlyCreatedMapPoints.begin(), recentlyCreatedMapPoints.end(),
            m_recentlyCreatedMapPointIds.begin(), m_recentlyCreatedMapPointIds.end(),
            mira::emplace_inserter(newTrackPoints));

        for (const auto& curId : newTrackPoints)
        {
            m_recentlyCreatedMapPointsScoring.emplace(curId.first, MapPointScore{ 0, 0 });
        }

        m_recentlyCreatedMapPointIds = move(recentlyCreatedMapPoints);
    }

    Pose TrackLocalMap::OptimizeCameraPose(
        const Pose& currentFramePose,
        const CameraCalibration& cameraCalibration,
        gsl::span<const ProjectedMapPoint> mapPoints,
        unsigned int numIterations,
        float maxOutlierErrorSquared,
        float huberWidth,
        mira::determinator& determinator,
        thread_memory memory,
        std::vector<unsigned int>& outlierIndices)
    {
        SCOPE_TIMER(TrackLocalMap::OptimizeCameraPose);
        (void)determinator;

        //Do Bundle adjustment

        //TODO: consider making this time based and run until out of time.

        std::unique_ptr<BundlerLib> ba;
        {
            SCOPE_TIMER(TrackLocalMap::BuildG2OWrapper);

            BundlerParameters bundlerParameters{};
            bundlerParameters.ArePointsFixed = true;

            ba = std::make_unique<BundlerLib>(bundlerParameters);
        }

        {
            SCOPE_TIMER(TrackLocalMap::BuildDataForG2O);

            Eigen::Matrix3f rotation;
            cv::cv2eigen(currentFramePose.GetRotationMatrix(), rotation);

            //add only current frame to poses.
            ba->AllocateCameras(1);
            ba->SetCameraPose(0,
                ToCMap(currentFramePose.GetViewSpacePosition()),
                ToCMap(rotation),
                ToCMap(cameraCalibration.GetLinearIntrinsics()), /*isFixed*/ false);

            //add points and connections
            ba->AllocateMapPoints(mapPoints.size());
            ba->AllocateObservations(mapPoints.size());
            for (int i = 0; i < (int)mapPoints.size(); ++i)
            {
                ba->SetMapPoint(i, ToCMap(mapPoints[i].MapPoint));

                float  info = 1.0f;

                // scale by the refinement count so that points with less bundle iterations end up with a smaller information matrix
                // smaller information matrix corresponds to a larger variance in the measurement
                uint64_t refinementCount = mapPoints[i].MapPointRefinementCount;

                info *= MapPointRefinementConfidence(refinementCount);

                ba->SetObservation(i, ToCMap(mapPoints[i].Projection), 0, i, info);
            }
        }

        {
            SCOPE_TIMER(TrackLocalMap::StepBundleAdjustment);
            vector<float> huberWidths(numIterations, huberWidth);
            ba->StepBundleAdjustment(huberWidths, maxOutlierErrorSquared, outlierIndices);
        }

        // update the pose, we only added current pose so the one we want is in index 0
        cv::Point3f position;
        Eigen::Matrix3f rotation;
        ba->GetPose(0, ToMap(position), ToMap(rotation));

        {
            SCOPE_TIMER(TrackLocalMap::DestroyG2OWrapper);
            ba.reset();
        }

        cv::Matx33f cvrot;
        cv::eigen2cv(rotation, cvrot);

        return Pose{ position, cvrot };
    }

    bool TrackLocalMap::IsGoodCandidate(
        const std::shared_ptr<const AnalyzedImage>& image,
        unsigned int imageBorder,
        const Pose& curFramePose,
        const Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics, proxy::ViewingData>::ViewT& point,
        float minDegreesBetweenCurrentViewAndMapPointView)
    {
        Projection projected = ProjectUndistorted(
            curFramePose.GetViewMatrix(), image->GetUndistortedCalibration().GetCameraMatrix(), point->GetPosition());

        cv::Point3f framePosition = curFramePose.GetWorldSpacePosition();
        cv::Vec3f frameForward = curFramePose.GetWorldSpaceForward();

        return IsGoodCandidate(image, gsl::narrow_cast<float>(imageBorder), projected, framePosition, frameForward, point, minDegreesBetweenCurrentViewAndMapPointView);
    }

    bool TrackLocalMap::IsGoodCandidate(
        const std::shared_ptr<const AnalyzedImage>& image,
        float imageBorder,
        const Projection& projectedPoint,
        const cv::Point3f& framePosition,
        const cv::Vec3f& frameForward,
        const Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics, proxy::ViewingData>::ViewT& point,
        float minDegreesBetweenCurrentViewAndMapPointView)
    {
        // 1) if behind the camera or not in the frame skip it
        if (projectedPoint.Distance < 0 || !image->PointWithinImageBorder(projectedPoint.Point, imageBorder))
            return false;

        // 2) Check the angle between the current view direction and the map points
        // normalized view direction

        //verify they are normalized
        assert(abs(Length(point->GetMeanViewingDirection()) - 1.0f) < NRM_EPSILON);
        assert(abs(Length(frameForward) - 1.0f) < NRM_EPSILON);
        float dotResult = point->GetMeanViewingDirection().dot(frameForward);

        //PERF: cache the calculation on the setting  (unchanging)
        if (dotResult < std::cos(mira::deg2rad(minDegreesBetweenCurrentViewAndMapPointView)))
            return false;

        // 3) Compute the map point distance and discard it if it's out of the
        // scale invariance region (dmin, dmax)
        cv::Point3f deltaPosition = point->GetPosition() - framePosition;
        float distanceSquare = deltaPosition.dot(deltaPosition);

        if (distanceSquare < (point->GetDMin() * point->GetDMin()) || (point->GetDMax() * point->GetDMax()) < distanceSquare)
            return false;

        return true;

    }

    mira::unique_vector<Id<MapPoint>> TrackLocalMap::GetRecentMapPointsThatFailScoring() const
    {
        SCOPE_TIMER(TrackLocalMap::GetRecentMapPointsThatFailScoring);

        mira::unique_vector<Id<MapPoint>> mapPointsThatFailScoring;
        for (auto& curMapPointScoring : m_recentlyCreatedMapPointsScoring)
        {
            //if a recent map point isn't found where it was predicted to be found by at least this ratio it is a candidate for culling
            // Add 1 to top and bottom so that when small number of samples (stderr large) we less striked on hitting threshold.
            // As samples becomes large it results it limits to the regular percentage
            if (((curMapPointScoring.second.Found + 1) / (float)(curMapPointScoring.second.Predicted + 1)) < m_trackLocalMapSettings.RecentMapPointPctSuccess)
            {
                mapPointsThatFailScoring.insert_presorted(curMapPointScoring.first);
            }
        }

        return mapPointsThatFailScoring;
    }

    bool TrackLocalMap::MatchMapPointToCurrentFrame(
        const std::shared_ptr<const AnalyzedImage>& image,
        const std::vector<bool>& unassociatedKeypointsMask,
        const cv::KeyPoint& mapPointKeypoint,
        const ORBDescriptor::Ref& mapPointDesc,
        const OrbMatcherSettings& orbMatcherSettings,
        const float matchSearchRadius,
        thread_memory memory,
        KeypointDescriptorIndex &keypointDescriptorIndex)
    {
        cv::DMatch match;
        if (RadiusMatch(
            mapPointKeypoint,
            nullptr,
            mapPointDesc,
            image->GetKeyPoints(),
            image->GetKeypointSpatialIndex(),
            &unassociatedKeypointsMask,
            image->GetDescriptors(),
            matchSearchRadius,
            orbMatcherSettings.MaxHammingDistance,
            orbMatcherSettings.MinHammingDifference,
            memory,
            match))
        {
            keypointDescriptorIndex = match.trainIdx;
            return true;
        }

        return false;
    }

    Pose TrackLocalMap::GetUpdatedKeyframePoseRelativeToBasis(
        const Pose& keyframePose,
        const Pose& originalBasisPose,
        const Pose& updatedBasisPose)
    {
        return{
            updatedBasisPose.GetInverseViewMatrix() *
            originalBasisPose.GetViewMatrix4x4() *
            keyframePose.GetInverseViewMatrix()
            };
    }
}
