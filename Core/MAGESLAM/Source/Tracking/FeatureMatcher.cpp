// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

// The feature matcher finds matches for the descriptors between two frames.

#include "FeatureMatcher.h"
#include "MageSettings.h"
#include "Data/Keyframe.h"

#include "Utils/Logging.h"
#include "BoW/BaseBow.h"

#include <iterator>
#include <intrin.h>

#include <opencv2/core/hal/hal.hpp>

namespace mage
{
    namespace
    {
        struct TrackMatchResult
        {
            size_t MatchIdx;
            int HammingDistance;
        };

        void TrackMatch(
            const mage::ORBDescriptor& descriptorLeft,
            const std::shared_ptr<const AnalyzedImage>& imageRight,
            size_t idxRight,
            const std::vector<bool>& rightMask,
            TrackMatchResult& bestMatch,
            TrackMatchResult& secondBestMatch,
            int maxHamming)
        {
            if (rightMask[idxRight])
            {
                int hamming_dist = GetDescriptorDistance(descriptorLeft, imageRight->GetDescriptor(idxRight));

                if (hamming_dist < maxHamming)
                {
                    if (hamming_dist < bestMatch.HammingDistance)
                    {
                        secondBestMatch = bestMatch;
                        bestMatch = { idxRight, hamming_dist };
                    }
                    else if (hamming_dist < secondBestMatch.HammingDistance)
                    {
                        secondBestMatch = { idxRight, hamming_dist };
                    }
                }
            }
        }
    }

    /*
    Match does a two way match between the orb descriptor distances (hamming distance).
    This will get used in Map Initialization and anywhere else where basic descriptor matching is needed.
    */
    unsigned int Match(
        const std::shared_ptr<const AnalyzedImage>& imageA,
        const std::shared_ptr<const AnalyzedImage>& imageB,
        const std::vector<bool>& imageAMask,
        const std::vector<bool>& imageBMask,
        size_t imageAMaskTrueCount,
        size_t imageBMaskTrueCount,
        int maxHammingDist,
        int minHammingDifference,
        std::vector<cv::DMatch> &goodMatches)
    {
        if (imageAMaskTrueCount == 0 || imageBMaskTrueCount == 0)
        {
            //TODO: may want to report that there were no features in an image
            goodMatches.clear();
            return 0;
        }

        SCOPE_TIMER(FeatureMatcher::Match);

        // TODO:  get rid of these four allocations

        std::vector<size_t> imageAUnassociatedIndexes;
        imageAUnassociatedIndexes.reserve(imageAMaskTrueCount);
        std::vector<size_t> imageBUnassociatedIndexes;
        imageBUnassociatedIndexes.reserve(imageBMaskTrueCount);

        cv::Mat descriptorMatrixA((int)imageAMaskTrueCount, 32, CV_8UC1);
        for (size_t i = 0, j = 0; i < imageAMask.size(); ++i)
        {
            if (imageAMask[i])
            {
                imageA->GetDescriptor(gsl::narrow_cast<int>(i)).CopyTo(descriptorMatrixA.row(gsl::narrow_cast<int>(j)));
                imageAUnassociatedIndexes.push_back(i);
                j++;
            }
        }
        assert(imageAUnassociatedIndexes.size() == imageAMaskTrueCount);

        cv::Mat descriptorMatrixB((int)imageBMaskTrueCount, 32, CV_8UC1);
        for (size_t i = 0, j = 0; i < imageBMask.size(); ++i)
        {
            if (imageBMask[i])
            {
                imageB->GetDescriptor(gsl::narrow_cast<int>(i)).CopyTo(descriptorMatrixB.row(gsl::narrow_cast<int>(j)));
                imageBUnassociatedIndexes.push_back(i);
                j++;
            }
        }
        assert(imageBUnassociatedIndexes.size() == imageBMaskTrueCount);


        // TODO:  this process is very slow
        std::vector<std::vector<cv::DMatch>> matches;
        std::vector<std::vector<cv::DMatch>> matches_backwards;

        auto matcher = cv::BFMatcher{ cv::NORM_HAMMING, false };
        matcher.radiusMatch(descriptorMatrixA, descriptorMatrixB, matches, (float)maxHammingDist, cv::noArray(), true);
        matcher.radiusMatch(descriptorMatrixB, descriptorMatrixA, matches_backwards, (float)maxHammingDist, cv::noArray(), true);

        // build up a lookup of the best backwards matches
        std::vector<int> bestBackwardsMatch(matches_backwards.size(), -1);
        for (unsigned int i = 0; i < matches_backwards.size(); i++)
        {
            const std::vector<cv::DMatch>& currentMatch = matches_backwards[i];

            if (currentMatch.empty())
                continue;

            if (currentMatch.size() > 1)
            {
                float matchDelta = currentMatch[1].distance - currentMatch[0].distance;
                if (matchDelta < (float)minHammingDifference)
                    continue;
            }

            const cv::DMatch& match = currentMatch[0];
            bestBackwardsMatch[match.queryIdx] = match.trainIdx;
        }

        goodMatches.reserve(matches.size());
        for (unsigned int i = 0; i < matches.size(); i++)
        {
            const std::vector<cv::DMatch>& currentMatch = matches[i];

            if (currentMatch.empty())
                continue;

            if (currentMatch.size() > 1)
            {
                float matchDelta = currentMatch[1].distance - currentMatch[0].distance;
                if (matchDelta < (float)minHammingDifference)
                    continue;
            }

            const cv::DMatch& match = currentMatch[0];
            // check the backwards match
            if (bestBackwardsMatch[match.trainIdx] == match.queryIdx)
            {
                goodMatches.emplace_back(
                    gsl::narrow_cast<int>(imageAUnassociatedIndexes[match.queryIdx]),
                    gsl::narrow_cast<int>(imageBUnassociatedIndexes[match.trainIdx]),
                    match.distance);
            }
        }

        //#define DEBUG_MATCHES
#ifdef DEBUG_MATCHES
        cv::Mat debugMatches;
        const auto& imgAKeypoints = imageA->GetKeyPoints();
        const auto& imgBKeypoints = imageB->GetKeyPoints();
        cv::drawMatches(imageA->GetDebugImage(), std::vector<cv::KeyPoint>(imgAKeypoints.begin(), imgAKeypoints.end()),
            imageB->GetDebugImage(), std::vector<cv::KeyPoint>(imgBKeypoints.begin(), imgBKeypoints.end()),
            std::vector<cv::DMatch>(goodMatches.begin(), goodMatches.end()), debugMatches);

        // what if we just use openCV brute force matching?
        cv::Mat debugBfMatches;
        cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> bfMatches;
        bfMatcher.match(descriptorMatrixA, descriptorMatrixB, bfMatches);
        cv::drawMatches(imageA->GetDebugImage(), std::vector<cv::KeyPoint>(imgAKeypoints.begin(), imgAKeypoints.end()),
            imageB->GetDebugImage(), std::vector<cv::KeyPoint>(imgBKeypoints.begin(), imgBKeypoints.end()),
            bfMatches, debugBfMatches);

        static int j = 0;
        j++;
#endif

        return gsl::narrow_cast<unsigned int>(goodMatches.size());
    }

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
        temp::vector<cv::DMatch> &goodMatches)
    {
        if (imageAMaskCount == 0 || imageBMaskCount == 0) return 0;

        const int maxHamming = maxHammingDist + 1;

        std::vector<ptrdiff_t> closeMatches;

        temp::vector<std::pair<size_t, size_t>> matches = memory.stack_vector<std::pair<size_t, size_t>>(std::min(imageAMaskCount, imageBMaskCount));
        for (size_t idxA = 0; idxA < imageAMask.size(); ++idxA)
        {
            if (imageAMask[idxA])
            {
                const mage::ORBDescriptor& descA = imageA->GetDescriptor(idxA);
                if (matcherB != nullptr)
                {
                    matcherB->QueryFeatures(descA, closeMatches);
                }
                else
                {
                    bagOfWords.QueryFeatures(descA, idB, closeMatches);
                }

                TrackMatchResult bestMatch{ std::numeric_limits<size_t>::max(), maxHamming };
                TrackMatchResult secondBestMatch{ std::numeric_limits<size_t>::max(), maxHamming };

                for (size_t j = 0; j < closeMatches.size(); ++j)
                {
                    size_t idxB = closeMatches.at(j);
                    TrackMatch(descA, imageB, idxB, imageBMask, bestMatch, secondBestMatch, maxHamming);
                }

                if (bestMatch.HammingDistance < maxHamming &&
                    (secondBestMatch.HammingDistance >= maxHamming || secondBestMatch.HammingDistance - bestMatch.HammingDistance >= minHammingDifference))
                {
                    matches.emplace_back(idxA, bestMatch.MatchIdx);
                }
            }
        }

        for (const auto& match : matches)
        {
            size_t idxB = match.second;
            const mage::ORBDescriptor& descB = imageB->GetDescriptor(idxB);
            if (matcherA != nullptr)
            {
                matcherA->QueryFeatures(descB, closeMatches);
            }
            else
            {
                bagOfWords.QueryFeatures(descB, idA, closeMatches);
            }

            TrackMatchResult bestMatch{ std::numeric_limits<size_t>::max(), maxHamming };
            TrackMatchResult secondBestMatch{ std::numeric_limits<size_t>::max(), maxHamming };

            for (size_t j = 0; j < closeMatches.size(); ++j)
            {
                size_t idxA = closeMatches.at(j);
                TrackMatch(descB, imageA, idxA, imageAMask, bestMatch, secondBestMatch, maxHamming);
            }

            if (bestMatch.HammingDistance < maxHamming &&
                bestMatch.MatchIdx == match.first &&
                (secondBestMatch.HammingDistance >= maxHamming || secondBestMatch.HammingDistance - bestMatch.HammingDistance >= minHammingDifference))
            {
                // Since we were doing reverse matching, we have to switch the fields on best match.
                goodMatches.emplace_back(gsl::narrow_cast<int>(bestMatch.MatchIdx), gsl::narrow_cast<int>(idxB), (float)bestMatch.HammingDistance);
            }
        }

        //#define DEBUG_MATCHES
#ifdef DEBUG_MATCHES
        cv::Mat debugMatches;
        const auto& imgAKeypoints = imageA->GetKeyPoints();
        const auto& imgBKeypoints = imageB->GetKeyPoints();
        cv::drawMatches(imageA->GetDebugImage(), std::vector<cv::KeyPoint>(imgAKeypoints.begin(), imgAKeypoints.end()),
            imageB->GetDebugImage(), std::vector<cv::KeyPoint>(imgBKeypoints.begin(), imgBKeypoints.end()),
            std::vector<cv::DMatch>(goodMatches.begin(), goodMatches.end()), debugMatches);

        static int j = 0;
        j++;
#endif

        return gsl::narrow_cast<unsigned int>(goodMatches.size());
    }

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
        std::vector<cv::DMatch> &goodMatches)
    {
        SCOPE_TIMER(FeatureMatcher::RadiusMatch);

        assert(goodMatches.size() == 0 && "output vector should be empty");

        std::vector<cv::DMatch> almostGoodMatches;

        for (ptrdiff_t indexQuery = 0; indexQuery < queryKeypoints.size(); indexQuery++)
        {
            if (queryKeypointsMask != nullptr && !(*queryKeypointsMask)[indexQuery])
                continue;

            const cv::Point2f* queryPositionOverride = queryKeypointPositionOverrides == nullptr ? &queryKeypoints[indexQuery].pt : &queryKeypointPositionOverrides->at(indexQuery);
            cv::DMatch bestMatch;
            if (RadiusMatch(
                queryKeypoints[indexQuery],
                queryPositionOverride,
                queryDescriptors[indexQuery],
                targetKeypoints,
                targetKeypointsIndex,
                targetKeypointsMask,
                targetDescriptors,
                radius,
                maxHammingDist,
                minHammingDifference,
                memory,
                bestMatch))
            {
                // the single-item version always returns 0 for queryIdx, so populate it with the index of queryKeypoints.
                bestMatch.queryIdx = indexQuery;
                almostGoodMatches.push_back(bestMatch);
            }
        }

        if (almostGoodMatches.size() > 1)
        {
            temp::vector<float> bestDistance = memory.stack_vector<float>();
            temp::vector<float> secondBestDistance = memory.stack_vector<float>();
            bestDistance.resize(targetKeypoints.size(), std::numeric_limits<float>::max());
            secondBestDistance.resize(targetKeypoints.size(), std::numeric_limits<float>::max());

            for (const auto& bestMatch : almostGoodMatches)
            {
                if (bestMatch.distance < bestDistance[bestMatch.trainIdx])
                {
                    secondBestDistance[bestMatch.trainIdx] = bestDistance[bestMatch.trainIdx];
                    bestDistance[bestMatch.trainIdx] = bestMatch.distance;
                }
                else if (bestMatch.distance < secondBestDistance[bestMatch.trainIdx])
                {
                    secondBestDistance[bestMatch.trainIdx] = bestMatch.distance;
                }
            }

            // Select best match
            for (const auto& match : almostGoodMatches)
            {
                if (match.distance == bestDistance[match.trainIdx] &&
                    bestDistance[match.trainIdx] < secondBestDistance[match.trainIdx])
                {
                    goodMatches.emplace_back(match);
                }
            }
        }
        else
        {
            goodMatches = almostGoodMatches;
        }

        return gsl::narrow_cast<unsigned int>(goodMatches.size());
    }

    /*
    RadiusMatch does matching based on distance from keypoints in image 1. It searches a radius around the keypoint from image1
    for a matching feature in image2. The match does a basic match based on hamming distance between the orb descriptors.
    The bestMatch parameter will be populated with queryIdx = 0, trainIdx = index into targetKeypoints, distance = hamming distance
    of match.
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
        cv::DMatch& bestMatch)
    {
        assert(queryKeypoint.octave >= 0);

        int bestHammingDistance = maxHammingDist + 1;
        int secondBestHammingDistance = std::numeric_limits<int>::max();
        bestMatch.trainIdx = -1;
        bestMatch.queryIdx = 0;
        bestMatch.imgIdx = 0;

        cv::Point2f position = queryKeypointPositionOverride == nullptr ? queryKeypoint.pt : *queryKeypointPositionOverride;

        temp::vector<size_t> resultIndexes = memory.stack_vector<size_t>(targetKeypoints.size());

        targetKeypointsIndex.Query(position, queryKeypoint.octave, radius, resultIndexes);

        for (const auto& indexTarget : resultIndexes)
        {
            assert(targetKeypoints[indexTarget].octave >= 0);

            if (targetKeypointsMask == nullptr || targetKeypointsMask->at(indexTarget))
            {
                /*
                imagePointsB contains the map points from A but in image B,
                that's why we use indexA to access it.
                */
                int hamming_dist = GetDescriptorDistance(*queryDescriptor, targetDescriptors[indexTarget]);
                if (hamming_dist < bestHammingDistance)
                {
                    // this is a possible match 
                    bestMatch.trainIdx = indexTarget;
                    // store the hamming distance between the matches 
                    bestMatch.distance = (float)hamming_dist;

                    secondBestHammingDistance = bestHammingDistance;

                    // found a new best distance, so store it
                    bestHammingDistance = hamming_dist;
                }
            }
        }

        // see if initialized, i.e. match found
        if (bestMatch.trainIdx != -1 && (secondBestHammingDistance - bestHammingDistance) > minHammingDifference)
        {
            return true;
        }
        return false;
    }

    int GetDescriptorDistanceSlow(const ORBDescriptor& d0, const ORBDescriptor& d1)
    {
        return cv::hal::normHamming(d0.Data(), d1.Data(), ORBDescriptor::COLUMNS);
    }

    int GetDescriptorDistance(const ORBDescriptor& d0, const ORBDescriptor& d1)
    {
        int result = 0;
#ifdef _M_ARM
        const uint8_t* a = (const uint8_t*)(d0.Data());
        const uint8_t* b = (const uint8_t*)(d1.Data());

        {
            uint8x16_t A_vec1 = vld1q_u8(a);
            uint8x16_t B_vec1 = vld1q_u8(b);
            uint8x16_t A_vec2 = vld1q_u8(a + 16);
            uint8x16_t B_vec2 = vld1q_u8(b + 16);

            uint32x4_t bits = vmovq_n_u32(0);

            {
                uint8x16_t AxorB = veorq_u8(A_vec1, B_vec1);
                uint8x16_t bitsSet = vcntq_u8(AxorB);
                uint16x8_t bitSet8 = vpaddlq_u8(bitsSet);
                uint32x4_t bitSet4 = vpaddlq_u16(bitSet8);
                bits = vaddq_u32(bits, bitSet4);
            }

            {
                uint8x16_t AxorB = veorq_u8(A_vec2, B_vec2);
                uint8x16_t bitsSet = vcntq_u8(AxorB);
                uint16x8_t bitSet8 = vpaddlq_u8(bitsSet);
                uint32x4_t bitSet4 = vpaddlq_u16(bitSet8);
                bits = vaddq_u32(bits, bitSet4);
            }

            uint64x2_t bitSet2 = vpaddlq_u32(bits);
            result = vgetq_lane_s32(vreinterpretq_s32_u64(bitSet2), 0);
            result += vgetq_lane_s32(vreinterpretq_s32_u64(bitSet2), 2);
        }
#else
        const uint32_t* data0 = (const uint32_t*)(d0.Data());
        const uint32_t* data1 = (const uint32_t*)(d1.Data());

        const uint32_t* data0end = data0 + 8;

        for (; data0 < data0end; data0++, data1++)
        {
            int bits = *data0^*data1;
            bits = bits - ((bits >> 1) & 0x55555555);
            bits = (bits & 0x33333333) + ((bits >> 2) & 0x33333333);
            result += (((bits + (bits >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
        }
#endif

        return result;
    }
}
