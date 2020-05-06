// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "NewMapPointsCreation.h"
#include "Map/MappingMath.h"
#include "Tracking/FeatureMatcher.h"
#include "Tracking/Reprojection.h"
#include "Tracking/TrackLocalMap.h"
#include "Tracking/Triangulation.h"
#include "Utils/Epipolar.h"
#include "Utils/cv.h"
#include "Utils/Logging.h"

#include <arcana/math.h>

#include "Analysis/DataPoints.h"
#include <arcana/analysis/object_trace.h>

namespace mage
{
    NewMapPointsCreationState::NewMapPointsCreationState(const BaseBow& bagOfWords, const KeyframeProxy& keyFrameId, const PerCameraSettings& camSettings, const mage::NewMapPointsCreationSettings& nmpcSettings, const float scale) :
        BagOfWords{ bagOfWords },
        Ki{ keyFrameId },
        KiViewMatrix{ Ki.GetPose().GetViewMatrix() },
        KiCameraMatrix{ Ki.GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix() },
        KiInverseViewMatrix{ Ki.GetPose().GetInverseViewMatrix() },
        KiPyramidScale{ Ki.GetAnalyzedImage()->GetPyramidScale() },
        KiNumLevels { Ki.GetAnalyzedImage()->GetNumLevels() },
        KiImageBorder { Ki.GetAnalyzedImage()->GetImageBorder() },
        CameraSettings { camSettings },
        NewMapPointsCreationSettings{ nmpcSettings },
        CosMinParallax{ cos(mira::deg2rad<float>(nmpcSettings.MinParallaxDegrees)) },
        MapScale(scale)
    {
    }

    bool VerifyProjectInFront(
        const cv::Matx34f& viewMatrix,
        const cv::Matx33f& cameraCalibration,
        const cv::Point3f& triangulatedPoint)
    {
        // TODO PERF since we don't actually care about the projection pixel position in this test, we could be more efficient and not call ProjectUndistorted directly
        // right now, this isn't called frequently enough for that to matter.

        // validation for new keyframe: positive depth
        auto newKeypointReprojectedUndistorted = ProjectUndistorted(viewMatrix, cameraCalibration, triangulatedPoint);
        if (newKeypointReprojectedUndistorted.Distance <= 0)
            return false;

        return true;
    }

    bool ParallaxTest(const cv::Vec3f& v1, const cv::Vec3f& v2, float cosMinParallax)
    {
        assert(abs(sqrtf(v1.dot(v1)) - 1) < 0.001f);
        assert(abs(sqrtf(v2.dot(v2)) - 1) < 0.001f);
        float cosActualParallax = v1.dot(v2);
        if (cosActualParallax > cosMinParallax)
        {
            return false;
        }

        return true;
    }

    // Validate the gemoetry of the provided match with respect to epipolarity, triangulation, scale,
    // and parallax.  If the validations all pass, then the point makes sense; make the point and 
    // associated it with the keyframes/descriptors used to create the point.
    //
    // This method modifies NewMapPointsCreationState and leaves it ready for future calls to 
    // either TryCreateNewAssociatedMapPointsForMatch() or LocallyAssociateNewAssociations().
    // It also modifies newMapPoints and keyframeAssociations.  Note that no modifications 
    // occur if the method returns false.
    bool TryCreateNewAssociatedMapPointForMatch(
        NewMapPointsCreationState& state,
        std::vector<MapPointKeyframeAssociations>& newMapPoints)
    {
        // Values computed for Epipolar Test, kept for later use.
        const cv::KeyPoint& KcKeypointUndistorted = state.Kc->GetAnalyzedImage()->GetKeyPoint(state.KcKeypointDescriptorIdx);
        const cv::KeyPoint& KiKeypointUndistorted = state.Ki.GetAnalyzedImage()->GetKeyPoint(state.KiKeypointDescriptorIdx);

        // Epipolar Test, done inline because it's trivial.
        const float pixelMaxEpipolarDistance = state.NewMapPointsCreationSettings.MaxEpipolarError;
        const float KcToKiEpipolarDistance = DistanceFromEpipolarLine(state.FundamentalMatrixKcToKi, KcKeypointUndistorted.pt, KiKeypointUndistorted.pt);
        const float KiToKcEpipolarDistance = DistanceFromEpipolarLine(state.FundamentalMatrixKiToKc, KiKeypointUndistorted.pt, KcKeypointUndistorted.pt);

        if (KiToKcEpipolarDistance + KcToKiEpipolarDistance > 2 * pixelMaxEpipolarDistance)
            return false;

        // Values computed for Triangulation Validation, kept for later use.
        const cv::Matx33f& KcCalInv = state.Kc->GetAnalyzedImage()->GetUndistortedCalibration().GetInverseCameraMatrix();
        const cv::Matx33f& KiCalInv = state.Ki.GetAnalyzedImage()->GetUndistortedCalibration().GetInverseCameraMatrix();
        const cv::Point3f triangulatedPt = TriangulatePointWorldSpace(KcCalInv, state.KcInverseViewMatrix, KiCalInv, state.KiInverseViewMatrix, KcKeypointUndistorted.pt, KiKeypointUndistorted.pt);

        // Validate that the points projects in front of the camera
        if (!VerifyProjectInFront(state.KiViewMatrix, state.KiCameraMatrix, triangulatedPt)
            || !VerifyProjectInFront(state.KcViewMatrix, state.KcCameraMatrix, triangulatedPt))
        {
            return false;
        }

        // Note that the paper indicates that we should check "reprojection error" during new map points creation
        // However, because we just used two KeyPoints to create the map point, there isn't anything to validate.
        // So, we use the bundle adjustment at the end to validate the map point reprojection and throw out points that
        // aren't consistent with the model

        // Values computed for Scale Test, kept for later use.
        cv::Vec3f KcToTrianglulatedPoint = triangulatedPt - state.Kc->GetPose().GetWorldSpacePosition();
        float KcToTriangulatedPointDistance = sqrtf(KcToTrianglulatedPoint.dot(KcToTrianglulatedPoint));
        cv::Vec3f KiToTriangulatedPoint = triangulatedPt - state.Ki.GetPose().GetWorldSpacePosition();
        float KiToTriangulatedPointDistance = sqrtf(KiToTriangulatedPoint.dot(KiToTriangulatedPoint));

        // Distance Ratio Test
        // To avoid creating points near the camera that will be culled later, we do a distance ratio test, where 
        // points that are near the keyframe, relative to the distance between the keyframes that see the point, are
        // not added.
        cv::Vec3f KiToKc = state.Ki.GetPose().GetWorldSpacePosition() - state.Kc->GetPose().GetWorldSpacePosition();
        float KiToKcDistance = sqrtf(KiToKc.dot(KiToKc));
        float distanceRatio = KiToTriangulatedPointDistance / KiToKcDistance;
        if (distanceRatio < state.NewMapPointsCreationSettings.MinAcceptedDistanceRatio)
        {
            return false;
        }

        // Scale Test (inline because it's trivial)
        int predictedOctave = ComputeOctave(KcToTriangulatedPointDistance,
            ComputeDMin(KiToTriangulatedPointDistance, KiKeypointUndistorted.octave, state.KiPyramidScale),
            state.KiPyramidScale);
        if (abs(predictedOctave - KcKeypointUndistorted.octave) >= 1)
            return false;

        // Values computed for the Parallax Test, kept for later use.
        cv::Vec3f KcToTriangulatedPtNormalizedDir = KcToTrianglulatedPoint / KcToTriangulatedPointDistance;
        cv::Vec3f KiToTriangulatedPtNormalizedDir = KiToTriangulatedPoint / KiToTriangulatedPointDistance;

        // Parallax test
        if (!ParallaxTest(KcToTriangulatedPtNormalizedDir, KiToTriangulatedPtNormalizedDir, state.CosMinParallax))
            return false;

        // If we've made it this far successfully, create the map point itself.

        // We choose Ki to be the representative keyframe for the new map point.  This information will be 
        // reevaluated and refined upon insertion into the actual map.
        cv::Vec3f meanViewDirection = cv::normalize(KcToTriangulatedPtNormalizedDir + KiToTriangulatedPtNormalizedDir);
        float dMax = ComputeDMax(KiToTriangulatedPointDistance, KiKeypointUndistorted.octave, gsl::narrow_cast<int>(state.KiNumLevels), state.KiPyramidScale);
        float dMin = ComputeDMin(KiToTriangulatedPointDistance, KiKeypointUndistorted.octave, state.KiPyramidScale);
        MapPointProxy newMapPoint = MapPointProxy::CreateNew(triangulatedPt, proxy::UpdateStatistics{ 0 }, proxy::ViewingData{ meanViewDirection, dMin, dMax }, state.Ki.GetAnalyzedImage()->GetDescriptor(state.KiKeypointDescriptorIdx).AsRef());

        // associate with these two frames
        KeyframeAssociation KiKeypointAssoc{ state.Ki.GetId(), state.KiKeypointDescriptorIdx };
        KeyframeAssociation KcKeypointAssoc{ state.Kc->GetId(), state.KcKeypointDescriptorIdx };

        std::vector<KeyframeAssociation> mapPointAssocs{ KiKeypointAssoc, KcKeypointAssoc };

        //add association to the list of associations used for construction later
        newMapPoints.push_back({ newMapPoint, mapPointAssocs });
        state.KiAssociatedDescriptors.insert(state.KiKeypointDescriptorIdx);

        return true;
    }

    // For each covisible keyframe, match all the unassociated descriptors in that keyframe
    // to the unassociated descriptors in the current keyframe.
    //     For each resulting match (after confirming that the new keyframe descriptor involved
    //     has not been associated by a previous iteration), try to create a map point for the
    //     match.
    //
    // This method modifies the NewMapPointsCreationState and should leave it ready to be passed
    // into LocallyAssociateNewAssociations().  It also modifies newMapPoints and 
    // keyframeAssociations.
    void CreateInitialAssociations(
        NewMapPointsCreationState& state,
        gsl::span<MappingKeyframe> covisibleKeyframes,
        std::vector<MapPointKeyframeAssociations>& newMapPoints,
        int maxHammingDist,
        int minHammingDifference,
        ptrdiff_t maxFrameToSearch,
        thread_memory memory)
    {
        SCOPE_TIMER(NewMapPointsCreation::CreateInitialAssociations);

        const size_t gridWidth = state.CameraSettings.NewPointGridWidth;
        const size_t gridHeight = state.CameraSettings.NewPointGridHeight;
        auto gridCount = memory.stack_vector<size_t>();
        gridCount.resize(gridWidth*gridHeight);

        for (size_t i = 0; i<state.Ki.GetAssociatedKeypointCount(); i++)
        {
            if (state.Ki.GetAssociatedKeypointMask()[i])
            {
                const auto& analyzedImage = state.Ki.GetAnalyzedImage();
                const auto& keypoint = analyzedImage->GetKeyPoint(i);

                size_t imageWidth = analyzedImage->GetWidth();
                size_t imageHeight = analyzedImage->GetHeight();

                int x = (int)(keypoint.pt.x * gridWidth / imageWidth);
                int y = (int)(keypoint.pt.y * gridHeight / imageHeight);
                gridCount[x + y * gridWidth]++;
            }
        }

        size_t unassociatedCount = state.Ki.GetUnassociatedKeypointCount();
        if (unassociatedCount == 0)
        {
            return;
        }

        const std::vector<bool> KiUnassociatedKeypointMask = state.Ki.GetUnassociatedKeypointMask();
        // If the newest keyframe (Ki) has no unassociated descriptors, no new map points can be created.

        const auto kiWorldSpacePosition = state.Ki.GetPose().GetWorldSpacePosition();

        // iterate over the covisibility and sort them based upon distance from Ki in world coordinates
        // this will bias the matched map points to use better pairs for triangulation which should result
        // in better initial placements for map points
        std::sort(covisibleKeyframes.begin(), covisibleKeyframes.end(), [kiWorldSpacePosition](const auto& lhs, const auto& rhs) {
            auto lhsDist = lhs.GetPose().GetWorldSpacePosition() - kiWorldSpacePosition;
            auto rhsDist = rhs.GetPose().GetWorldSpacePosition() - kiWorldSpacePosition;
            return lhsDist.dot(lhsDist) < rhsDist.dot(rhsDist);
        });

        float mapScaleSquared = state.MapScale * state.MapScale;

        FIRE_OBJECT_TRACE("NewMapPointsCreation.NumCovisibleKeyframes", nullptr, make_frame_data_point(state.Ki, (float)covisibleKeyframes.size()));

        for (ptrdiff_t KcIdx = 0, frameCount = 0; KcIdx < covisibleKeyframes.size() && frameCount < maxFrameToSearch; KcIdx++)
        {
            state.SetKc(covisibleKeyframes[KcIdx]);

            // If the current covisible frame has no unassociated features, ignore it.
            if (state.Kc->GetUnassociatedKeypointCount() == 0)
            {
                continue;
            }

            // If the current frame is too close to the candidate frame, don't use it for initial triangulation
            auto keyframeDistance = state.Kc->GetPose().GetWorldSpacePosition() - kiWorldSpacePosition;
            float distSquared = keyframeDistance.dot(keyframeDistance);
            if (distSquared < (state.NewMapPointsCreationSettings.MinKeyframeDistanceForCreatingMapPointsSquared * mapScaleSquared))
            {
                continue;
            }

#ifndef NDEBUG
            size_t acceptedMatchCount = 0;
#endif
            frameCount++;

            temp::vector<cv::DMatch> matches = memory.stack_vector<cv::DMatch>(unassociatedCount);
            {
                SCOPE_TIMER(CreateInitialAssociations::IndexedMatch);

                int num_matches = IndexedMatch(
                    state.BagOfWords,
                    state.Kc->GetId(),
                    state.Ki.GetId(),
                    state.Kc->GetAnalyzedImage(),
                    state.Ki.GetAnalyzedImage(),
                    state.Kc->GetUnassociatedKeypointMask(),
                    KiUnassociatedKeypointMask,
                    state.Kc->GetUnassociatedKeypointCount(),
                    unassociatedCount,
                    maxHammingDist,
                    minHammingDifference,
                    memory,
                    matches);

                FIRE_OBJECT_TRACE(("NewMapPointsCreation.NumInitialMatches_Frame_" + std::to_string(frameCount - 1)).c_str(), nullptr, make_frame_data_point(state.Ki, (float)num_matches));

                if (num_matches == 0)
                {
                    continue;
                }
            }

            const auto KiCalibration = state.Ki.GetAnalyzedImage()->GetUndistortedCalibration();
            const auto KcCalibration = state.Kc->GetAnalyzedImage()->GetUndistortedCalibration();

            state.FundamentalMatrixKcToKi = ComputeFundamentalMatrix(state.Kc->GetPose(), KcCalibration, state.Ki.GetPose(), KiCalibration);
            state.FundamentalMatrixKiToKc = ComputeFundamentalMatrix(state.Ki.GetPose(), KiCalibration, state.Kc->GetPose(), KcCalibration);

            const auto& analyzedImage = state.Ki.GetAnalyzedImage();
            int imageWidth = analyzedImage->GetWidth();
            int imageHeight = analyzedImage->GetHeight();
            
            float gridWidthPixels = (float)gridWidth / (float)imageWidth;
            float gridHeightPixels = (float)gridHeight / (float)imageHeight;

            for (const cv::DMatch& match : matches)
            {
                state.KcKeypointDescriptorIdx = match.queryIdx;
                state.KiKeypointDescriptorIdx = match.trainIdx;

                const auto& keypoint = analyzedImage->GetKeyPoint(state.KiKeypointDescriptorIdx);

                size_t gridX = mira::clamp<size_t>((size_t)floor(keypoint.pt.x * gridWidthPixels), 0, gridWidth - 1);
                size_t gridY = mira::clamp<size_t>((size_t)floor(keypoint.pt.y * gridHeightPixels), 0, gridHeight - 1);
                
                size_t gridIndex = gridX + gridY * gridWidth;
                if (gridCount[gridIndex] >= state.CameraSettings.NewPointMaxGridCount)
                {
                    //all ready enough map points in this region, move onto next match.
                    continue;
                }

                // If no map point has been created for the matched feature in the newest keyframe,
                // make a new map point from this match.
                if (state.KiAssociatedDescriptors.find(state.KiKeypointDescriptorIdx) == state.KiAssociatedDescriptors.end())
                {
                    // If the new map point could be successfully created, memoize that this descriptor is now associated.
                    if (TryCreateNewAssociatedMapPointForMatch(state, newMapPoints))
                    {
                        covisibleKeyframes[KcIdx].SetKeypointAsAssociated(state.KcKeypointDescriptorIdx);
                        gridCount[gridIndex]++;
#ifndef NDEBUG
                        acceptedMatchCount++;
#endif
                    }
                }
            }

#ifndef NDEBUG
            FIRE_OBJECT_TRACE(("NewMapPointsCreation.NumAcceptedMatches_Frame_" + std::to_string(frameCount - 1)).c_str(), nullptr, make_frame_data_point(state.Ki, (float)acceptedMatchCount));
#endif
        }

        FIRE_OBJECT_TRACE("NewMapPointsCreation.NumNewPoints", nullptr, make_frame_data_point(state.Ki, (float)newMapPoints.size()));
    }

    // For each new map point created in CreateInitialAssociations()
    //     For each covisible keyframe that isn't already associated with this map point, attempt to
    //     match one of the unassociated descriptors of the keyframe with the map point.  If you succeed,
    //     Create the association.
    //
    // This method modifies NewMapPointsCreationState and keyframeAssociations.
    void LocallyAssociateNewAssociations(
        NewMapPointsCreationState& state,
        gsl::span<MappingKeyframe> covisibleKeyframes,
        thread_memory memory,
        std::vector<MapPointKeyframeAssociations>& newMapPoints)
    {
        SCOPE_TIMER(NewMapPointsCreation::LocallyAssociateNewAssociations);

        const float mapBorder = state.KiImageBorder -
            state.NewMapPointsCreationSettings.NewMapPointsSearchRadius / 2.0f;

#ifndef NDEBUG
        std::map<mage::Id<Keyframe>, size_t> newCovisConnections;
        for (const MappingKeyframe& Kc : covisibleKeyframes)
        {
            newCovisConnections[Kc.GetId()] = 0;
        }
#endif

        for (MapPointKeyframeAssociations& newMapPoint : newMapPoints)
        {
            for (ptrdiff_t KcIdx = 0; KcIdx < covisibleKeyframes.size(); KcIdx++)
            {
                const MappingKeyframe& Kc = covisibleKeyframes[KcIdx];

                const cv::Matx33f& cameraMat = Kc.GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix();
                const cv::Matx34f viewMat = Kc.GetPose().GetViewMatrix();

                // If this is the keyframe that the first association was made using, do not attempt to re-make the association.
                auto found = find_if(newMapPoint.Keyframes.begin(), newMapPoint.Keyframes.end(),
                    [=](const KeyframeAssociation& assoc) { return assoc.KeyframeId == Kc.GetId(); });

                if (found == newMapPoint.Keyframes.end())
                {
                    cv::Point3f framePosition = Kc.GetPose().GetWorldSpacePosition();
                    cv::Vec3f frameForward = Kc.GetPose().GetWorldSpaceForward();

                    Projection projected = ProjectUndistorted(viewMat, cameraMat, newMapPoint.MapPoint.GetPosition());

                    if (TrackLocalMap::IsGoodCandidate(Kc.GetAnalyzedImage(),
                        mapBorder,
                        projected,
                        framePosition,
                        frameForward,
                        newMapPoint.MapPoint,
                        state.NewMapPointsCreationSettings.MaxKeyframeAngleDegrees))
                    {
                        cv::Point3f deltaPosition = newMapPoint.MapPoint.GetPosition() - framePosition;
                        float distanceSquare = deltaPosition.dot(deltaPosition);
                        //PERF: the octave is already computed during IsGoodCandidate, no need to compute it again here
                        int octave = ComputeOctave(sqrtf(distanceSquare), newMapPoint.MapPoint.GetDMin(), state.KiPyramidScale);
                        // the predicted octave might be outside the possible range, if so this means that a match isn't possible,
                        // no point in continuing to evaluate this map point
                        // note that this shouldn't happen because the points should get thrown out by IsGoodCandidate
                        // however, in practice this will occur due to floating point rounding on occasion
                        if (octave < 0 || octave > static_cast<int>(state.KiNumLevels))
                        {
                            continue;
                        }

                        KeypointDescriptorIndex foundKeypointDescriptorIndex = std::numeric_limits<KeypointDescriptorIndex>::max();
                        cv::KeyPoint mapPointKp(projected.Point, -1.0f, 0.0f, 0.0f, octave, -1);
                        if (TrackLocalMap::MatchMapPointToCurrentFrame(
                            Kc.GetAnalyzedImage(),
                            covisibleKeyframes[KcIdx].GetUnassociatedKeypointMask(),
                            mapPointKp,
                            newMapPoint.MapPoint.GetRepresentativeDescriptor(),
                            state.NewMapPointsCreationSettings.AssociateMatcherSettings, state.NewMapPointsCreationSettings.NewMapPointsSearchRadius,
                            memory,
                            foundKeypointDescriptorIndex))
                        {
                            newMapPoint.Keyframes.push_back(KeyframeAssociation{ Kc.GetId(), foundKeypointDescriptorIndex });
                            covisibleKeyframes[KcIdx].SetKeypointAsAssociated(foundKeypointDescriptorIndex);
#ifndef NDEBUG
                            newCovisConnections[Kc.GetId()]++;
#endif
                        }
                    }
                }
            }
        }
#ifndef NDEBUG
        for (ptrdiff_t KcIdx = 0; KcIdx < covisibleKeyframes.size(); KcIdx++)
        {
            FIRE_OBJECT_TRACE(("NewMapPointsCreation.NumLocallyAssociated_Frame_" + std::to_string(KcIdx)).c_str(), nullptr, make_frame_data_point(state.Ki, (float)newCovisConnections[covisibleKeyframes[KcIdx].GetId()]));
        }
#endif
    }

    void NewMapPointsCreation(
        const BaseBow& bagOfWords,
        const KeyframeProxy& Ki,
        const PerCameraSettings& cameraSettings,
        const NewMapPointsCreationSettings& newMapPointsSettings,
        const float scale,
        thread_memory memory,
        gsl::span<MappingKeyframe> covisibleKeyframes,
        std::vector<MapPointKeyframeAssociations>& newMapPoints)
    {
        SCOPE_TIMER(NewMapPointsCreation::NewMapPointsCreation);

        NewMapPointsCreationState state(bagOfWords, Ki, cameraSettings, newMapPointsSettings, scale);

        CreateInitialAssociations(state, covisibleKeyframes, newMapPoints,
            newMapPointsSettings.InitialMatcherSettings.MaxHammingDistance,
            newMapPointsSettings.InitialMatcherSettings.MinHammingDifference,
            newMapPointsSettings.MaxFramesForNewPointsCreation, memory);

        LocallyAssociateNewAssociations(state, covisibleKeyframes, memory, newMapPoints);
    }
}
