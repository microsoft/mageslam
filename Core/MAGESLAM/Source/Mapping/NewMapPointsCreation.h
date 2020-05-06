// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Bow\BaseBow.h"
#include "Tracking\TrackLocalMap.h"
#include "Image\AnalyzedImage.h"

#include <gsl\gsl>

#include <memory>
#include <vector>

namespace mage
{    
    // VI-C : LOCAL MAPPING - NEW MAP POINT CREATION
    // create new map points for keyframe by triangulate ORB from connected keyframes KC in covisibility graph
    void NewMapPointsCreation(
        const BaseBow& bagOfWords,
        const KeyframeProxy& Ki,
        const PerCameraSettings& cameraSettings,
        const NewMapPointsCreationSettings& newMapPointSettings,
        const float scale,
        thread_memory memory,
        gsl::span<MappingKeyframe> Kcs,
        std::vector<MapPointKeyframeAssociations>& newMapPoints);

    // Carries the state, arguments, and cached computations through the NewMapPointsCreation operation.
    // Because this struct carries the state, operations that use its contents may depend on prior 
    // operations to populate that content; in other words, any method that takes a NewMapPointsCreationState
    // is expected to modify the state in such a way that it is ready for the next call.
    // used internally by new map points creation
    struct NewMapPointsCreationState
    {
        const BaseBow& BagOfWords;
        const KeyframeProxy& Ki;                        // new keyframe
        const MappingKeyframe* Kc;                      // connected keyframe
        size_t KcKeypointDescriptorIdx;                 // index into the Kc analyzed image's keypoints/descriptors
        size_t KiKeypointDescriptorIdx;                 // index into the Ki analyzed image's keypoints/descriptors
        cv::Matx33f FundamentalMatrixKcToKi;            // fundamental matrix Kc to Ki
        cv::Matx33f FundamentalMatrixKiToKc;            // fundamental matrix Ki to Kc
        cv::Matx34f KiViewMatrix;                       // new keyframe matrix
        cv::Matx33f KiCameraMatrix;                     // Camera Calibration for Ki
        cv::Matx44f KiInverseViewMatrix;                // new keyframe world matrix
        cv::Matx34f KcViewMatrix;                       // connected keyframe matrix
        cv::Matx33f KcCameraMatrix;                     // Camera Calibration for Kc
        cv::Matx44f KcInverseViewMatrix;                // connected keyframe world matrix
        std::set<size_t> KiAssociatedDescriptors;       // new keyframe associated descriptors
        const float KiPyramidScale;                           // Pyramid Scale used when features were extracted of Ki
        const float KiImageBorder;                            // Image Border for which no keypoints could have been extracted
        const size_t KiNumLevels;                             // Number of levels used to extract features in Ki

        const PerCameraSettings& CameraSettings;
        const NewMapPointsCreationSettings& NewMapPointsCreationSettings;
        const float CosMinParallax;
        const float MapScale;

        NewMapPointsCreationState(const BaseBow& bagofWords, const KeyframeProxy& keyFrameId, const PerCameraSettings& cameraSettings, const mage::NewMapPointsCreationSettings& newMapPointsCreationSettings, const float scale);

        void SetKc(const MappingKeyframe& kc)
        {
            Kc = &kc;
            KcInverseViewMatrix = Kc->GetPose().GetInverseViewMatrix();
            KcViewMatrix = Kc->GetPose().GetViewMatrix();
            KcCameraMatrix = Kc->GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix();
        }
    };

    // tests whether a match should be turned into a mappoint, if so creates the new map point
    // used internally by new map points creation
    bool TryCreateNewAssociatedMapPointForMatch(
        NewMapPointsCreationState& state,
        std::vector<MapPointKeyframeAssociations>& keyframeAssociations);

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
        std::vector<MapPointKeyframeAssociations>& keyframeAssociations);

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
        thread_memory memory);

    bool VerifyProjectInFront(
        const cv::Matx34f& viewMatrix,
        const cv::Matx33f& cameraCalibration,
        const cv::Point3f& triangulatedPoint);
}
