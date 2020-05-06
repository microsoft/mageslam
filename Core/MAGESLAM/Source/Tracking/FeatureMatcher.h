// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

// The feature matcher finds matches for the descriptors between two frames.

#pragma once

#include "Image\KeypointSpatialIndex.h"
#include "Image\ORBDescriptor.h"
#include "Data\Types.h"

#include <opencv2\features2d\features2d.hpp>

namespace mage
{
    class BaseBow;
    class BaseFeatureMatcher;
    class Keyframe;
    class AnalyzedImage;

    /*
    Globally matches descriptors based on their hamming distance, using QueryFeatures to speed up the
    results.  Will still work if CanQueryFeatures is false, but will be slower.  If only one is
    queryable, it is faster to have it in slot A.

    The accuracy of the match is determined by DirectIndexLevels.  A value of 6 will make it
    return all possible results.  Each integer less will cut the search space by 10, but will miss some
    possible matches.  A value of 4 is recommended.
    */
    unsigned int IndexedMatch(
        const BaseBow& bagOfWords,
        const BaseFeatureMatcher* matcherA,
        const BaseFeatureMatcher* matcherB,
        const Id<Keyframe> idA,
        const Id<Keyframe> idB,
        const std::shared_ptr<const AnalyzedImage>& imageA,
        const std::shared_ptr<const AnalyzedImage>& imageB,
        const std::vector<bool>& imageAMask,
        const std::vector<bool>& imageBMask,
        size_t imageAMaskCount,
        size_t imageBMaskCount,
        int maxHammingDist,
        int minHammingDifference,
        thread_memory& memory,
        temp::vector<cv::DMatch> &goodMatches);

    inline unsigned int IndexedMatch(
        const BaseBow& bagOfWords,
        const Id<Keyframe> idA,
        const Id<Keyframe> idB,
        const std::shared_ptr<const AnalyzedImage>& imageA,
        const std::shared_ptr<const AnalyzedImage>& imageB,
        const std::vector<bool>& imageAMask,
        const std::vector<bool>& imageBMask,
        size_t imageAMaskCount,
        size_t imageBMaskCount,
        int maxHammingDist,
        int minHammingDifference,
        thread_memory& memory,
        temp::vector<cv::DMatch> &goodMatches)
    {
        return IndexedMatch(bagOfWords, nullptr, nullptr, idA, idB, imageA, imageB, imageAMask, imageBMask, imageAMaskCount, imageBMaskCount, maxHammingDist, minHammingDifference, memory, goodMatches);
    };

    /*
        Globally matches descriptors based on their hamming distance.
    */
    unsigned int Match(
        const std::shared_ptr<const AnalyzedImage>& imageA,
        const std::shared_ptr<const AnalyzedImage>& imageB,
        const std::vector<bool>& imageAMask,
        const std::vector<bool>& imageBMask,
        size_t imageAMaskCount,
        size_t imageBMaskCount,
        int maxHammingDist,
        int minHammingDifference,
        std::vector<cv::DMatch> &goodMatches);

    /*
        Matches keypoints in a query set (eg from one captured frame)
        with keypoints in a target set (eg from another captured frame),
        by constraining matches to their location.

        keypointsAPositions is an optional parameter that has a new
        position for each keypoint, typically a reprojection from one
        image into another based on motion model or mappoint.  null
        means use the position from queryKeypoints.

        targetKeypointsMask is a set of points that are permissible
        to match against.  null means all.
    */
    unsigned int RadiusMatch(
        gsl::span<const cv::KeyPoint> queryKeypoints,
        const std::vector<cv::Point2f>* queryKeypointPositionOverrides,
        const std::vector<bool>* queryKeypointsMask,
        gsl::span<const ORBDescriptor::Ref> queryDescriptors,
        gsl::span<const cv::KeyPoint> targetKeypoints,
        const KeypointSpatialIndex& targetKeypointsIndex,
        const std::vector<bool>* targetKeypointsMask,
        gsl::span<const ORBDescriptor> targetDescriptors,
        float radius,
        int maxHammingDist,
        int minHammingDifference,
        thread_memory memory,
        std::vector<cv::DMatch>& goodMatches);

    /*
    Matches a single query keypoint (eg from one captured frame)
    with keypoints in a target set (eg from another captured frame),
    by constraining matches to their location.

    keypointsAPosition is an optional parameter that has a new
    position for the keypoint, typically a reprojection from one
    image into another based on motion model or mappoint.  null
    means use the position from queryKeypoint.

    targetKeypointsMask is a set of points that are permissible
    to match against.  null means all.
    */
    bool RadiusMatch(
        const cv::KeyPoint& queryKeypoint,
        const cv::Point2f* queryKeypointPositionOverride,
        const ORBDescriptor::Ref& queryDescriptor,
        gsl::span<const cv::KeyPoint> targetKeypoints,
        const KeypointSpatialIndex& targetKeypointsIndex,
        const std::vector<bool>* targetKeypointsMask,
        gsl::span<const ORBDescriptor> targetDescriptors,
        float radius,
        int maxHammingDist,
        int minHammingDifference,
        thread_memory memory,
        cv::DMatch &bestMatch);

    int GetDescriptorDistanceSlow(const ORBDescriptor& d0, const ORBDescriptor& d1);

    int GetDescriptorDistance(const ORBDescriptor& d0, const ORBDescriptor& d1);

}
