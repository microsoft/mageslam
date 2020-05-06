// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Tracking\ComputeEssential.h"
#include "Tracking\MapInitialization.h"
#include "Mapping\NewMapPointsCreation.h"
#include "Tracking\Triangulation.h"
#include "Tracking\Reprojection.h"
#include "MageSettings.h"
#include "Utils\Epipolar.h"
#include "Map\Map.h"
#include "Map\MapPoint.h"
#include "KeyframeBuilder.h"
#include "Utils\format.h"
#include "Utils\cv.h"
#include "FeatureMatcher.h"
#include "BundleAdjustment\BundleAdjust.h"
#include "Debugging\SkeletonLogger.h"
#include "arcana\analysis\determinator.h"
#include "Analysis\binary_iterators.h"

#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>

#include "Utils\Logging.h"

#include <boost\format.hpp>

#include <chrono>
#include <assert.h>

using namespace std;

namespace mage
{
    namespace
    {
        float ComputePerpDistanceSquared(
            const cv::Matx31f& line,
            const cv::Matx31f& point)
        {
            auto A = line(0, 0);
            auto B = line(1, 0);
            auto C = line(2, 0);
            auto X = point(0, 0);
            auto Y = point(1, 0);

            auto numerator = (A * X) + (B*Y) + C;
            auto denominatorSquared = (A*A) + (B*B);

            return numerator * numerator / denominatorSquared;
        }

        long long ComputeFrameTimeDifference(const shared_ptr<const AnalyzedImage>& frame1, const shared_ptr<const AnalyzedImage>& frame2)
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(frame2->GetTimeStamp() - frame1->GetTimeStamp()).count();
        }

        void CreateNewMapPointsHelper(InitializationData& initializationData, mage::temp::vector<MappingKeyframe>& Kcs, const KeyframeProxy& Ki, BaseBow& bagOfWords, const MonoMapInitializationSettings& settings, const PerCameraSettings& cameraSettings, thread_memory memory)
        {
            // compute the current scale for the initialization pair
            float scale = static_cast<float>(cv::norm(initializationData.Frames.back()->GetPose().GetViewSpacePosition(), cv::NORM_L2));
            float minParallaxScale = settings.MapInitializationNewPointsCreationMinDistance * scale;

            vector<MapPointKeyframeAssociations> newMapPoints;
            NewMapPointsCreation(bagOfWords, Ki, cameraSettings,
                settings.NewMapPointsCreationSettings,
                minParallaxScale, memory, Kcs, newMapPoints);

            //associate new map points with the keyframes it is visible in
            for (const auto& curMapPoint : newMapPoints)
            {
                initializationData.MapPoints.emplace_back(curMapPoint.MapPoint);
                for (const KeyframeAssociation& curKeyframeAssoc : curMapPoint.Keyframes)
                {
                    auto found = std::find_if(initializationData.Frames.begin(), initializationData.Frames.end(),
                        [curKeyframeAssoc](const auto& keyframe) { return keyframe->GetId() == curKeyframeAssoc.KeyframeId; });
                    assert(found != initializationData.Frames.end());
                    (*found)->AddAssociation(curMapPoint.MapPoint.As<MapPointTrackingProxy>(), curKeyframeAssoc.KeypointDescriptorIndex);
                }
            }
        }
    }

    vector<Pose> MapInitialization::FindEssentialPotientialPoses(
        const cv::Matx33f& essentialMatrix)
    {
        cv::Mat R1, R2;
        cv::Mat T;

        //This gives us back 2 possible rotations, and a translation that can be negative
        // From the docs:
        //      Generally 4 possible poses exists for a given E.
        //      They are[R1, t], [R1, -t], [R2, t], [R2, -t].
        cv::decomposeEssentialMat(essentialMatrix, R1, R2, T);

        const cv::Vec3f normalizedTranslationViewSpace = Normalize(cv::Vec3f{ T });
        const cv::Vec3f negT = -1 * normalizedTranslationViewSpace;

        vector<Pose> poses;
        poses.reserve(4);

        //pose of R1, T
        Pose p1 = { normalizedTranslationViewSpace, R1 };
        poses.emplace_back(p1);
        poses.emplace_back(negT, R1);

        //pose of R2, T
        Pose p2 = { T, R2 };
        poses.emplace_back(p2);
        poses.emplace_back(negT, R2);

        return poses;
    }

    MapInitialization::MapInitialization(
        const MonoMapInitializationSettings& settings,
        const PerCameraSettings& cameraSettings,
        const device::IMUCharacterization& imuCharacterization,
        mira::determinator& determinator)
        :   m_settings{ settings },
            m_cameraSettings{ cameraSettings },
            m_imuCharacterization{ imuCharacterization },
            m_determinator{ determinator }
    {
        ResetMapInitialization();
    }

    bool HasConsistentCheirality(const vector<Pose>& frame2Poses, /*cv::Vec3f& dominantDirection,*/
        const CameraCalibration& frame1Calibration, const CameraCalibration& frame2Calibration,
        const vector<cv::Point2f>& frame1Points, const vector<cv::Point2f>& frame2Points)
    {
        assert(frame1Points.size() == frame2Points.size());

        // Find any of the poses that have the expected orientation, to simplify cheirality computations.
        auto found = find_if(frame2Poses.begin(), frame2Poses.end(), [](const Pose& p) { return p.GetWorldSpaceRight()[0] > 0; });
        if (found == frame2Poses.end())
            return false;
        const Pose& frame2Pose = *found;

        // First frame is always at origin and identity.
        const Pose frame1Pose;

        const cv::Matx33f& frame1InvCam = frame1Calibration.GetInverseCameraMatrix();
        const cv::Matx33f& frame2InvCam = frame2Calibration.GetInverseCameraMatrix();
        const cv::Matx44f& frame1World = frame1Pose.GetInverseViewMatrix();
        const cv::Matx44f& frame2World = frame2Pose.GetInverseViewMatrix();

        const cv::Vec3f frame1Forward = frame1Pose.GetWorldSpaceForward();
        const cv::Vec3f frame2Forward = frame2Pose.GetWorldSpaceForward();

        int concensus = 1;
        {
            const cv::Point3f current3DPoint = TriangulatePointWorldSpace(frame1InvCam, frame1World, frame2InvCam, frame2World,
                frame1Points[0], frame2Points[0]);

            const cv::Vec3f fromFrame1 = current3DPoint - frame1Pose.GetWorldSpacePosition();
            const cv::Vec3f fromFrame2 = current3DPoint - frame2Pose.GetWorldSpacePosition();

            if (frame1Forward.dot(fromFrame1) <= 0.f || frame2Forward.dot(fromFrame2) <= 0.f)
                concensus = -1;
        }

        for (size_t pointIterator = 1; pointIterator < frame1Points.size(); pointIterator++)
        {
            const cv::Point3f current3DPoint = TriangulatePointWorldSpace(frame1InvCam, frame1World, frame2InvCam, frame2World,
                frame1Points[pointIterator], frame2Points[pointIterator]);

            const cv::Vec3f fromFrame1 = current3DPoint - frame1Pose.GetWorldSpacePosition();
            const cv::Vec3f fromFrame2 = current3DPoint - frame2Pose.GetWorldSpacePosition();

            if (concensus * frame1Forward.dot(fromFrame1) <= 0.f || concensus * frame2Forward.dot(fromFrame2) <= 0.f)
                return false;
        }

        //dominantDirection = frame2Pose.GetWorldSpacePosition() * concensus;
        
        return true;
    }

    vector<Pose> MapInitialization::FindPossiblePoses(
        gsl::span<const cv::Point2f> frame1Points,
        gsl::span<const cv::Point2f> frame2Points,
        const CameraCalibration& frame1Calibration,
        const CameraCalibration& frame2Calibration)
    {
        SCOPE_TIMER(MapInitialization::FindPossiblePoses);
        struct InitialPoseEstimate
        {
            float Score;
            std::vector<Pose> Candidates;
        };

        InitialPoseEstimate bestFive{ 0.0f, {} };

        cv::RNG rng(12345);

        const size_t pointCount = 5;

        std::vector<cv::Point2f> frame1Points_subset;
        std::vector<cv::Point2f> frame2Points_subset;
        frame1Points_subset.reserve(pointCount);
        frame2Points_subset.reserve(pointCount);

        const cv::Point2f principalPoint{ frame1Calibration.GetPrincipalPointX(),frame1Calibration.GetPrincipalPointY() };
        const cv::Matx33f inverseCalibrationCamera2Transpose = frame2Calibration.GetInverseCameraMatrix().t();
        const cv::Matx33f inverseCalibrationCamera1 = frame1Calibration.GetInverseCameraMatrix();
        
        const float minPixelDiffSquared = m_settings.MinPixelSpread * m_settings.MinPixelSpread;
        for (unsigned int i = 0; i < m_settings.RansacIterationsForModels; i++)
        {
            frame1Points_subset.clear();
            frame2Points_subset.clear();
            
            size_t tryCount = 0;
            std::set<int> randomIndices;
            while (randomIndices.size() < pointCount && tryCount < 100)
            {
                tryCount++;
                int candidateIndex = rng.uniform(0, frame1Points.size());

                bool tooClose = std::any_of(randomIndices.cbegin(), randomIndices.cend(), [&](int value)
                {
                    const auto pixelDiff1 = frame1Points[value] - frame1Points[candidateIndex];
                    const float dist2_1 = pixelDiff1.dot(pixelDiff1);

                    const auto pixelDiff2 = frame2Points[value] - frame2Points[candidateIndex];
                    const float dist2_2 = pixelDiff2.dot(pixelDiff2);

                    return dist2_1 < minPixelDiffSquared || dist2_2 < minPixelDiffSquared;
                });

                if (!tooClose)
                {
                    randomIndices.insert(candidateIndex);
                }
            }

            // if we weren't able to find a set of points, reset and try again
            if (randomIndices.size() < pointCount)
            {
                continue;
            }

            for (auto index : randomIndices)
            {
                frame1Points_subset.push_back(frame1Points[index]);
                frame2Points_subset.push_back(frame2Points[index]);
            }

            // Essential Matrix estimation (5 point)
            cv::Mat essentialMat1To2 = mira::FindEssentialMat(frame1Points_subset, frame2Points_subset, frame1Calibration.GetFocalLengthX(), frame1Calibration.GetFocalLengthY(), principalPoint, 1e-6f);

            // openCV may actually compute several essential matrixes
            int rowIterator = 0;
            while (essentialMat1To2.rows - rowIterator >= 3)
            {
                cv::Matx33f ess = cv::Matx33f{ essentialMat1To2.rowRange(rowIterator, rowIterator + 3) };
                cv::Matx33f fundamental1to2 = inverseCalibrationCamera2Transpose * ess * inverseCalibrationCamera1;
                cv::Matx33f fundamental2to1 = fundamental1to2.t();
                float fiveScore = ScoreFundamentalMatrix(frame1Points, frame2Points, fundamental1to2, fundamental2to1);

                if (fiveScore > bestFive.Score)
                {
                    auto possiblePoses = FindEssentialPotientialPoses(ess);
                    if (possiblePoses.size() > 0 && HasConsistentCheirality(possiblePoses, frame1Calibration, frame2Calibration, frame1Points_subset, frame2Points_subset))
                    {
                        bestFive = { fiveScore, possiblePoses };
                    }
                }

                rowIterator += 3;
            }
        }

        return bestFive.Candidates;
    }

    float MapInitialization::ScoreFundamentalMatrix(
        gsl::span<const cv::Point2f> frame1Points,
        gsl::span<const cv::Point2f> frame2Points,
        const cv::Matx33f& fundamentalMat1To2,
        const cv::Matx33f& fundamentalMat2To1)
    {
        float Total_F_sum = 0;
        size_t inlierCount = 0;

        //compute the difference between:
        //      point1 * (Fundamental Matrix) vs actual points 2
        //      point2 * (Fundamental Matrix Transpose) vs actual points 1
        for (auto frame1 = frame1Points.begin(), frame2 = frame2Points.begin();
            frame1 != frame1Points.end() && frame2 != frame2Points.end(); ++frame1, ++frame2)
        {
            cv::Matx31f  m1{ frame1->x, frame1->y, 1 };
            cv::Matx31f  m2{ frame2->x, frame2->y, 1 };

            cv::Matx31f result12 = fundamentalMat1To2 * m1;
            cv::Matx31f result21 = fundamentalMat2To1 * m2;

            float diff12Squared = ComputePerpDistanceSquared(result12, m2);
            float diff21Squared = ComputePerpDistanceSquared(result21, m1);

            //only add to the sum if the squared difference is less than the inlier threshold
            if (diff12Squared < m_settings.FundamentalTransferErrorThreshold
                && diff21Squared < m_settings.FundamentalTransferErrorThreshold)
            {
                Total_F_sum += (m_settings.FundamentalTransferErrorThreshold - diff12Squared);
                Total_F_sum += (m_settings.FundamentalTransferErrorThreshold - diff21Squared);
                inlierCount++;
            }
        }

        if (inlierCount >= m_settings.MinScoringInliers
            && (float)inlierCount / (float)frame1Points.size() > m_settings.MinInlierPercentage)
        {
            return Total_F_sum;
        }
        else
        {
            return 0;
        }
    }

    bool MapInitialization::FindCorrectPose(
        gsl::span<const cv::Point2f> frame1Points,
        gsl::span<const cv::Point2f> frame2Points,
        gsl::span<const cv::DMatch> matches,
        const CameraCalibration& frame1Calibration,
        const CameraCalibration& frame2Calibration,
        const Pose& referencePose,
        const vector<Pose>& poses,
        Pose& correctPose,
        std::vector<std::pair<cv::DMatch, cv::Point3f>>& triangulated3Dpoints)
    {
        SCOPE_TIMER(MapInitialization::FindCorrectPose);
        assert(frame1Points.size() == frame2Points.size());
        assert(triangulated3Dpoints.size() == 0);

        cv::Matx34f firstPoseMatrix = referencePose.GetViewMatrix();
        float maxEpipolarError = 2 * m_settings.MaxEpipolarError;

        float nextBestPoseScore = 0;
        float selectedPoseScore = 0;
        int selectedPoseIndex = -1;

        vector<pair<cv::DMatch, cv::Point3f>> selectedPosePoints;
        vector<pair<cv::DMatch, cv::Point3f>> candidatePosePoints;
        candidatePosePoints.reserve(frame1Points.size());
        selectedPosePoints.reserve(frame1Points.size());
        for (uint poseIterator = 0; poseIterator < poses.size(); poseIterator++)
        {
            candidatePosePoints.clear();

            const Pose& currentPose = poses.at(poseIterator);

            // From a given essential matrix, there are four possible solutions for the relative
            // camera pose; two of these will have a sensible rotation, and two will be "twisted"
            // a half-turn around the first camera's forward vector (see Nister's five-point paper,
            // section 3.1).  Because we know that such a twist will never occur in the brief 
            // interval between our initialization frames, we can early-out if we detect that we're
            // dealing with one of the twisted poses.
            if (currentPose.GetWorldSpaceRight().dot(referencePose.GetWorldSpaceRight()) <= 0.f)
            {
                continue;
            }

            const cv::Matx33f fundamentalMat = ComputeFundamentalMatrix(referencePose, frame1Calibration, currentPose, frame2Calibration);
            const cv::Matx33f fundamentalMat2 = ComputeFundamentalMatrix(currentPose, frame2Calibration, referencePose, frame1Calibration);

            //Triangulate the points from the first and second frame with the first (identity) matrix and one of the possible poses.
            const cv::Matx33f& frame1InvCam = frame1Calibration.GetInverseCameraMatrix();
            const cv::Matx33f& frame2InvCam = frame2Calibration.GetInverseCameraMatrix();
            const cv::Matx44f& frame1World = referencePose.GetInverseViewMatrix();
            const cv::Matx44f& frame2World = currentPose.GetInverseViewMatrix();

            // compute a scale factor so that the distance between the computed poses can be "1" for calculations
            assert(referencePose.GetWorldSpacePosition().x == 0 && "referencePose should be identity");
            assert(referencePose.GetWorldSpacePosition().y == 0 && "referencePose should be identity");
            assert(referencePose.GetWorldSpacePosition().z == 0 && "referencePose should be identity");
            const float scale = 1.0f / currentPose.GetWorldSpacePosition().dot(currentPose.GetWorldSpacePosition());

            float score = 0;
            for (int pointIterator = 0; pointIterator < frame1Points.size(); pointIterator++)
            {
                const cv::Point3f current3DPoint = TriangulatePointWorldSpace(frame1InvCam, frame1World, frame2InvCam, frame2World,
                    frame1Points[pointIterator], frame2Points[pointIterator]);

                // we only want to accept points which don't project out too far in the Z direction
                // we can get away with just testing the Z direction because the first pose is identity
                // but we do need to compute a normalization for the frame pair for this test to be reasonable

                // test that the point is in front
                if (current3DPoint.z < 0)
                {
                    continue;
                }

                // parallax test
                if (current3DPoint.z * scale > m_settings.MaxParallax3dDistance)
                {
                    continue;
                }

                const float KcToKiEpipolarError = DistanceFromEpipolarLine(fundamentalMat, frame1Points[pointIterator], frame2Points[pointIterator]);
                const float KiToKcEpiploarError = DistanceFromEpipolarLine(fundamentalMat2, frame2Points[pointIterator], frame1Points[pointIterator]);

                float temp = KiToKcEpiploarError + KcToKiEpipolarError;

                if (temp < maxEpipolarError)
                {
                    score += maxEpipolarError - temp;

                    candidatePosePoints.emplace_back(matches[pointIterator], current3DPoint);
                }
            }
            
            assert(score >= 0);

            if (candidatePosePoints.size() >= m_settings.MinScoringInliers
                && (float)candidatePosePoints.size() / (float)frame1Points.size() > m_settings.MinInlierPercentage)
            {
                //better parallax test
                {
                    size_t middleIndex = candidatePosePoints.size() / 2;
                    std::nth_element(candidatePosePoints.begin(), candidatePosePoints.begin() + middleIndex, candidatePosePoints.end(),
                        [](const pair<cv::DMatch, cv::Point3f>& matchAndPoint1, const pair<cv::DMatch, cv::Point3f>& matchAndPoint2)
                    {
                        return matchAndPoint1.second.z < matchAndPoint2.second.z;
                    });
                    auto medianDepth = candidatePosePoints[middleIndex].second.z;

                    // ensure that the depths of the traingulated points match the expected 'good' range
                    if (medianDepth <= m_settings.MaxParallax3dMedianDistance)
                    {
                        if (selectedPoseIndex == -1 || score > selectedPoseScore)
                        {
                            nextBestPoseScore = selectedPoseScore;
                            selectedPoseScore = score;
                            selectedPoseIndex = poseIterator;
                            std::swap(selectedPosePoints, candidatePosePoints);
                        }
                        else
                        {
                            // handle the case that this score isn't best, but is better than next best score
                            nextBestPoseScore = max(nextBestPoseScore, score);
                        }
                    }
                }
            }
        }

        // None of the potiential poses are valid
        if (selectedPoseScore <= 0)
        {
            LogMessage<Tracing::TraceLevels::Verbose>(L"MapInitialization  No pose with valid score");
            return false;
        }

        // compute the difference between the two best pose scores, if they are too similar, reject the pairing
        float scoreRatio = (selectedPoseScore - nextBestPoseScore) / selectedPoseScore;
        if (scoreRatio < m_settings.MinCandidatePoseDisimilarity)
        {
            LogMessage<Tracing::TraceLevels::Verbose>(L"MapInitialization  Best poses too similar");
            return false;
        }

        // validate for z-direction constraint
        if (abs(poses.at(selectedPoseIndex).GetWorldSpacePosition().z) > m_settings.MaxPoseContributionZ)
        {
            LogMessage<Tracing::TraceLevels::Verbose>(L"MapInitialization  Best pose too biased in Z direction");
            return false;
        }

        // At least one of the poses is valid, and the most accurate is chosen
        {
            correctPose = poses.at(selectedPoseIndex);
            triangulated3Dpoints = selectedPosePoints;
            return true;
        }
    }

    void MapInitialization::ResetMapInitialization()
    {
        LogMessage<>(L"MapInitialization  ResetMapInitialization");
        m_initializationFrames.clear();
        m_initializationDescriptorsCounters.clear();
    }

    MapInitialization::InitializationAttemptState MapInitialization::TryInitializeMap(
        const shared_ptr<const AnalyzedImage>& frame,
        thread_memory memory,
        InitializationData& initializationData,
        BaseBow& bagOfWords)
    {
        SCOPE_TIMER(MapInitialization::TryInitializeMap);

        // ensure that too much time hasn't passed, if so then re-init
        if (m_initializationFrames.size() != 0)
        {
            long long frameTimeDeltaMillisecondsFront = ComputeFrameTimeDifference(m_initializationFrames.front().Image, frame);
            // reset initialization
            if (frameTimeDeltaMillisecondsFront > m_settings.MaxInitializationIntervalMilliseconds)
            {
                ResetMapInitialization();
            }
            else
            {
                // deterministic skipping of frames during initialization period
                // if we are trying to compute poses, then only accept frames which are beyond the interval time delta
                if (frameTimeDeltaMillisecondsFront > m_settings.MinInitializationIntervalMilliseconds)
                {
                    long long frameTimeDeltaMillisecondsBack = ComputeFrameTimeDifference(m_initializationFrames.back().Image, frame);
                    if (frameTimeDeltaMillisecondsBack < m_settings.MapInitFrameIntervalMilliseconds)
                    {
                        return InitializationAttemptState::Skipped;
                    }
                }
            }
        }

        // add the new frame to our list of frames
        m_initializationFrames.emplace_back(frame);

        // bootstrap just the call and set things up
        if (m_initializationFrames.size() == 1)
        {
            unsigned int numDescriptors = m_initializationFrames.front().Image->GetDescriptors().size();

            if (numDescriptors < m_settings.MinFeatureMatches)
            {
                // not enough matches, reset and try again
                ResetMapInitialization();
                return InitializationAttemptState::ResetInit;
            }
            m_initializationDescriptorsCounters.clear();
            m_initializationDescriptorsCounters.resize(numDescriptors, 1); // each descriptor has been seen in one frame

            return InitializationAttemptState::NoPose;
        }
        LogMessage<Tracing::TraceLevels::Verbose>((boost::wformat(L"MapInitialization frame %d Initializing has %d frames") % frame->GetFrameId().CorrelationId % m_initializationFrames.size()).str());

        if (!TryIntializeMapWithProvidedFrames(m_initializationFrames.front(), m_initializationFrames.back(), memory, initializationData, bagOfWords))
        {
            return InitializationAttemptState::NoPose;
        }

        return InitializationAttemptState::FinishInit;
    }

    bool MapInitialization::TryIntializeMapWithProvidedFrames(
        const MatchedImage& referenceFrame,
        MatchedImage& currentFrame,
        thread_memory memory,
        InitializationData& initializationData,
        BaseBow& bagOfWords)
    {
        SCOPE_TIMER(MapInitialization::TryIntializeMapWithProvidedFrames);

        // match points with the descriptors that have been valid over the set of initialization frames
        //QUERY IMAGE: referenceFrame
        //TRAIN IMAGE: current frame
        int descriptorSizeFrame1 = gsl::narrow_cast<int>(referenceFrame.Image->GetDescriptorsCount());
        int trueCountFrame1 = 0;
        vector<bool> flagsFrame1(descriptorSizeFrame1, false);
        for (int i = 0; i < descriptorSizeFrame1; i++)
        {
            if (referenceFrame.Image->GetKeyPoint(i).octave == 0)
            {
                flagsFrame1[i] = true;
                trueCountFrame1 += 1;
            }
        }
        int descriptorSizeFrame2 = gsl::narrow_cast<int>(currentFrame.Image->GetDescriptorsCount());
        int trueCountFrame2 = 0;
        vector<bool> flagsFrame2(descriptorSizeFrame2, false);
        for (int i = 0; i < descriptorSizeFrame2; i++)
        {
            if (currentFrame.Image->GetKeyPoint(i).octave == 0)
            {
                flagsFrame2[i] = true;
                trueCountFrame2 += 1;
            }
        }

        unsigned int numMatches = Match(referenceFrame.Image,
            currentFrame.Image, flagsFrame1, flagsFrame2, trueCountFrame1, trueCountFrame2,
            m_settings.FivePointMatchingSettings.MaxHammingDistance,
            m_settings.FivePointMatchingSettings.MinHammingDifference, currentFrame.Matches);

        SkeletonLogger::MapInitialization::LogMatches(currentFrame.Matches, *referenceFrame.Image, *currentFrame.Image);

        if (numMatches < m_settings.MinFeatureMatches)
        {
            SkeletonLogger::MapInitialization::LogFailure("numMatches < m_mapInitializationSettings.MinFeatureMatches");

            ResetMapInitialization();
            return false;
        }
        // increment the counter for each feature that we matched
        for (unsigned int row = 0; row < numMatches; row++)
        {
            m_initializationDescriptorsCounters[currentFrame.Matches[row].queryIdx]++;
        }

        long long frameTimeDeltaMilliseconds = ComputeFrameTimeDifference(referenceFrame.Image, currentFrame.Image);
        size_t frameCount = m_initializationFrames.size();
        // if the frame count crosses the threshold, then we have a candidate match to try and initialize with
        if (frameTimeDeltaMilliseconds >= m_settings.MinInitializationIntervalMilliseconds)
        {
            // check to make sure that enough of the common features were seen in many of the frames which we have matched
            vector<cv::DMatch> bestMatches;

            assert(frameCount * m_settings.FeatureCovisibilityThreshold < 256);
            uint8_t goodFeatureThreshold = (uint8_t)(frameCount * m_settings.FeatureCovisibilityThreshold);
            
            for (unsigned int row = 0; row < numMatches; row++)
            {
                const auto& currentMatch = currentFrame.Matches[row];
                if (m_initializationDescriptorsCounters[currentMatch.queryIdx] > goodFeatureThreshold)
                {
                    bestMatches.push_back(currentMatch);
                }
            }

            // ensure that we still have enough matches
            if (bestMatches.size() < m_settings.MinFeatureMatches)
            {
                ResetMapInitialization();
                SkeletonLogger::MapInitialization::LogFailure("bestMatches.size() < m_mapInitializationSettings.MinFeatureMatches");
                LogMessage<Tracing::TraceLevels::Verbose>((boost::wformat(L"MapInitialization  Not enough matches : %d") % bestMatches.size()).str());
                return false;
            }

            // try to compute the pose for the frame pairing
            bool initializeSucceeded = InitializeWithFrames(bestMatches, referenceFrame.Image, currentFrame.Image, initializationData, memory);
            LogMessage<Tracing::TraceLevels::Verbose>((boost::wformat(L"MapInitialization  Initializing attempt success? %s") % (initializeSucceeded ? "true" : "false")).str());
            SkeletonLogger::MapInitialization::LogFailure(initializeSucceeded ? "Initialize succeeded" : "Initialize failed");

            if (!initializeSucceeded)
            {
                return false;
            }

            DETERMINISTIC_CHECK(m_determinator, initializationData.Frames.begin(), initializationData.Frames.end());
            DETERMINISTIC_CHECK(m_determinator, initializationData.MapPoints.begin(), initializationData.MapPoints.end());

            // perform new map points creation before bundling this solution to add additional datapoints
            // clear out any existing map points, because we're going to add new ones
            initializationData.MapPoints.clear();
            for (auto& keyframe : initializationData.Frames)
            {
                keyframe->ClearAssociations();
            }
            std::unique_ptr<BaseBow> tempBagOfWords = bagOfWords.CreateTemporaryBow();

            for (const auto& keyframe : initializationData.Frames)
            {
                tempBagOfWords->AddImage(keyframe->GetId(), *keyframe->GetAnalyzedImage());
            }
            
            // Create a list of connected keyframes which in this case is just thhe first frame
            mage::temp::vector<MappingKeyframe> Kcs = memory.stack_vector<MappingKeyframe>(initializationData.Frames.size());
            Kcs.emplace_back(*initializationData.Frames.front());

            CreateNewMapPointsHelper(initializationData, Kcs, *initializationData.Frames.back(), *tempBagOfWords, m_settings, m_cameraSettings, memory);

            // Validate that we were able to create a minimal set of map points using the NewMapPointsCreation technique
            if ((float)initializationData.MapPoints.size() < m_settings.MinInitialMapPoints)
            {
                return false;
            }

            // perform an initial bundle adjust of the map before creating new map points
            BundlerSettings bundlerSettings1{
                m_settings.BundleAdjustmentHuberWidth,
                m_settings.MaxOutlierError,
                1.0f,
                0.0f,
                false,
                m_settings.BundleAdjustmentG2OSteps,
                m_settings.BundleAdjustmentG2OSteps,
                0 };

            BundleAdjustInitializationData(initializationData, m_determinator, true, bundlerSettings1, memory);

            DETERMINISTIC_CHECK(m_determinator, initializationData.Frames.begin(), initializationData.Frames.end());
            DETERMINISTIC_CHECK(m_determinator, initializationData.MapPoints.begin(), initializationData.MapPoints.end());

            // re-validate that we have enough map points following post-BA outlier culling
            if ((float)initializationData.MapPoints.size() < m_settings.MinInitialMapPoints)
            {
                return false;
            }

            // now we'll add in a third frame and its associations to the existing set of map points
            bool addThirdFrame = m_initializationFrames.size() > 2;
            if (addThirdFrame)
            {
                SCOPE_TIMER(MapInitialization::InitializeWithFrames::LocateThirdFrame);
                
                const auto& frame0 = initializationData.Frames.front();
                const auto& frame1 = initializationData.Frames.back();
                auto& thirdFrame = initializationData.Frames[1];
                const auto& image0 = frame0->GetAnalyzedImage();
                const auto& image1 = frame1->GetAnalyzedImage();
                const auto& image_thirdFrame = thirdFrame->GetAnalyzedImage();
                const Pose& pose0 = frame0->GetPose();
                const Pose& pose1 = frame1->GetPose();

                // need to estimate a pose for this new frame.  For now just assume in the middle of the initialization pair
                const mage::Quaternion worldRotationQuat = ToQuat(Rotation(pose1.GetInverseViewMatrix()));
                const cv::Vec3f halfPosition = (pose0.GetWorldSpacePosition() + pose1.GetWorldSpacePosition()) / 2;
                const mage::Quaternion halfRotation = ToQuat(Rotation(pose0.GetInverseViewMatrix())).slerp(0.5f, worldRotationQuat);
                Pose additionalPose{ To4x4(ToMat(halfRotation), halfPosition) };

                const cv::Matx33f& calibration = image_thirdFrame->GetUndistortedCalibration().GetCameraMatrix();
                std::vector<bool> unassociatedKeypointsMask = std::vector<bool>(image_thirdFrame->GetDescriptorsCount(), true);
                auto mapPoints = memory.stack_vector<TrackLocalMap::ProjectedMapPoint>(initializationData.MapPoints.size());
                auto mapPointIds = memory.stack_vector<Id<MapPoint>>(initializationData.MapPoints.size());

                // project the triangulated point back into the third frame and
                // perform a radius match to find the best matching point
                for (const auto& mapPoint : initializationData.MapPoints)
                {
                    const mage::Projection projection = mage::ProjectUndistorted(additionalPose.GetViewMatrix(), calibration, mapPoint.GetPosition());

                    // skip point projected behind the image
                    if (projection.Distance < 0) continue;

                    auto keypointIndex = frame0->GetAssociatedIndex(mapPoint);

                    cv::DMatch referenceMatch;
                    const cv::KeyPoint mapPointKeypoint(projection.Point, -1.0f, 0.0f, 0.0f, 0, -1);
                    bool referenceMatched = RadiusMatch(
                        mapPointKeypoint,
                        nullptr,
                        ORBDescriptor::Ref{ image0->GetDescriptor(keypointIndex) },
                        image_thirdFrame->GetKeyPoints(),
                        image_thirdFrame->GetKeypointSpatialIndex(),
                        &unassociatedKeypointsMask,
                        image_thirdFrame->GetDescriptors(),
                        m_settings.ExtraFrame_SearchRadius,
                        m_settings.ExtraFrameMatchingSettings.MaxHammingDistance,
                        m_settings.ExtraFrameMatchingSettings.MinHammingDifference,
                        memory,
                        referenceMatch);
                    referenceMatch.queryIdx = gsl::narrow_cast<int>(keypointIndex);

                    keypointIndex = frame1->GetAssociatedIndex(mapPoint);
                    cv::DMatch currentMatch;
                    bool currentMatched = RadiusMatch(
                        mapPointKeypoint,
                        nullptr,
                        ORBDescriptor::Ref{ image1->GetDescriptor(keypointIndex) },
                        image_thirdFrame->GetKeyPoints(),
                        image_thirdFrame->GetKeypointSpatialIndex(),
                        &unassociatedKeypointsMask,
                        image_thirdFrame->GetDescriptors(),
                        m_settings.ExtraFrame_SearchRadius,
                        m_settings.FivePointMatchingSettings.MaxHammingDistance,
                        m_settings.FivePointMatchingSettings.MinHammingDifference,
                        memory,
                        currentMatch);
                    currentMatch.queryIdx = gsl::narrow_cast<int>(keypointIndex);

                    // if only one match, use that match
                    // if both matched, but to different keypoints then choose the closest match
                    if (currentMatched && referenceMatched)
                    {
                        if (currentMatch.distance < referenceMatch.distance)
                        {
                            unassociatedKeypointsMask[currentMatch.trainIdx] = false;
                            mapPoints.emplace_back(mapPoint.GetPosition(), image_thirdFrame->GetKeyPoint(currentMatch.trainIdx).pt, 1);
                            mapPointIds.emplace_back(mapPoint.GetId());
                            thirdFrame->AddAssociation(mapPoint, currentMatch.trainIdx);
                        }
                        else
                        {
                            unassociatedKeypointsMask[referenceMatch.trainIdx] = false;
                            mapPoints.emplace_back(mapPoint.GetPosition(), image_thirdFrame->GetKeyPoint(referenceMatch.trainIdx).pt, 1);
                            mapPointIds.emplace_back(mapPoint.GetId());
                            thirdFrame->AddAssociation(mapPoint, referenceMatch.trainIdx);
                        }
                    }
                    else if (currentMatched)
                    {
                        unassociatedKeypointsMask[currentMatch.trainIdx] = false;
                        mapPoints.emplace_back(mapPoint.GetPosition(), image_thirdFrame->GetKeyPoint(currentMatch.trainIdx).pt, 1);
                        mapPointIds.emplace_back(mapPoint.GetId());
                        thirdFrame->AddAssociation(mapPoint, currentMatch.trainIdx);
                    }
                    else if (referenceMatched)
                    {
                        unassociatedKeypointsMask[referenceMatch.trainIdx] = false;
                        mapPoints.emplace_back(mapPoint.GetPosition(), image_thirdFrame->GetKeyPoint(referenceMatch.trainIdx).pt, 1);
                        mapPointIds.emplace_back(mapPoint.GetId());
                        thirdFrame->AddAssociation(mapPoint, referenceMatch.trainIdx);
                    }
                }

                // if the middle initialization frame isn't adding enough value then abort this init
                // and look for a better trio of frames
                if ((float)thirdFrame->GetAssociatedKeypointCount() / (float)initializationData.MapPoints.size() < m_settings.MinThirdFrameMatchPercentage)
                {
                    return false;
                }

                // perform a track local map like bundle adjust of only this third frame to better align it
                // with the map points that were match above without adjusting the points and culling outliers
                std::vector<unsigned int> outliers;
                outliers.reserve(mapPoints.size());
                const float outlierErrorSquared = m_settings.ExtraFrame_MaxOutlierError * m_settings.ExtraFrame_MaxOutlierError;
                Pose updatedPose;
                {
                    SCOPE_TIMER(MapInitialization::OptimizeThirdCameraPose);
                    updatedPose = TrackLocalMap::OptimizeCameraPose(
                        additionalPose,
                        image_thirdFrame->GetUndistortedCalibration(),
                        mapPoints,
                        m_settings.ExtraFrame_BundleAdjustmentSteps,
                        outlierErrorSquared,
                        m_settings.ExtraFrame_HuberWidth,
                        m_determinator,
                        memory,
                        outliers);
                }

                // Cull outliers
                for (const size_t outlierIndex : outliers)
                {
                    thirdFrame->RemoveAssociation(mapPointIds[outlierIndex]);
                }

                // recheck that outlier culling hasn't dropped our match percentage too low
                if ((float)thirdFrame->GetAssociatedKeypointCount() / (float)initializationData.MapPoints.size() < m_settings.MinThirdFrameMatchPercentage)
                {
                    return false;
                }

                initializationData.Frames[1]->SetPose(additionalPose);
                // Create a list of connected keyframes of all of the previous inputed frames
                // note that Kcs is already expected to contain the 'front' element
                assert(Kcs.size() == 1 && "Kcs should contain only one element");
                assert(Kcs.front().GetId() == initializationData.Frames.front()->GetId() && "Kcs should contain the first element");
                Kcs.emplace_back(*initializationData.Frames.back());

                // use the middle frame as the seed for creating new map points
                assert(initializationData.Frames.size() == 3 && "Expecting three frame initialization");
                CreateNewMapPointsHelper(initializationData, Kcs, *initializationData.Frames[1], *tempBagOfWords, m_settings, m_cameraSettings, memory);
                
                DETERMINISTIC_CHECK(m_determinator, initializationData.Frames.begin(), initializationData.Frames.end());
                DETERMINISTIC_CHECK(m_determinator, initializationData.MapPoints.begin(), initializationData.MapPoints.end());

                BundlerSettings finalBundlerSettings{
                    m_settings.FinalBA_HuberWidth,
                    m_settings.FinalBA_MaxOutlierError,
                    m_settings.FinalBA_MaxOutlierErrorScaleFactor,
                    m_settings.FinalBA_MinMeanSquareError,
                    false,
                    m_settings.FinalBA_NumStepsPerRun,
                    m_settings.FinalBA_NumSteps,
                    0 };

                BundleAdjustInitializationData(initializationData, m_determinator, true, finalBundlerSettings, memory);

                DETERMINISTIC_CHECK(m_determinator, initializationData.Frames.begin(), initializationData.Frames.end());
                DETERMINISTIC_CHECK(m_determinator, initializationData.MapPoints.begin(), initializationData.MapPoints.end());

            }

            if (!ValidateInitializationData(initializationData,
                m_settings.MaxPoseContributionZ,
                m_settings.AmountBACanChangePose,
                m_settings.MinMapPoints))
            {
                DETERMINISTIC_CHECK(m_determinator, 8794548);
                return false;
            }

            LogMessage<Tracing::TraceLevels::Verbose>((boost::wformat(L"MapInitialization  Initializing attempt success? %s") % (initializeSucceeded ? "true" : "false")).str());

            SkeletonLogger::MapInitialization::LogFailure(initializeSucceeded ? "Initialize succeeded" : "Initialize failed");

            return initializeSucceeded;
        }
        return false;
    }

    void MapInitialization::CollectMatchPoints(
        const shared_ptr<const AnalyzedImage>& referenceFrame,
        const shared_ptr<const AnalyzedImage>& currentFrame,
        gsl::span<const cv::DMatch> matches,
        vector<cv::Point2f>& frame1_points,
        vector<cv::Point2f>& frame2_points)
    {
        frame1_points.reserve(matches.size());
        frame2_points.reserve(matches.size());

        const auto& current_kp = currentFrame->GetKeyPoints();
        const auto& ref_kp = referenceFrame->GetKeyPoints();
        for (const auto& match : matches)
        {
            cv::Point2f point1 = ref_kp[match.queryIdx].pt;
            cv::Point2f point2 = current_kp[match.trainIdx].pt;

            frame1_points.push_back(point1);
            frame2_points.push_back(point2);
        }
    }

    void MapInitialization::TriangulatePoints(const shared_ptr<const AnalyzedImage>& referenceFrame, 
                                              const shared_ptr<const AnalyzedImage>& currentFrame,
                                              const Pose& referencePose,
                                              const Pose& currentPose,
                                              gsl::span<const cv::DMatch> matches,
                                              gsl::span<const cv::Point2f> frame1_points,
                                              gsl::span<const cv::Point2f> frame2_points,
                                              float pixelMaxEpipolarDistance,
                                              float minAcceptanceDistanceRatio,
                                              std::vector<pair<cv::DMatch, cv::Point3f>>& initial3DPoints)
    {
        const cv::Matx33f& frame1InvCam = referenceFrame->GetUndistortedCalibration().GetInverseCameraMatrix();
        const cv::Matx33f& frame2InvCam = currentFrame->GetUndistortedCalibration().GetInverseCameraMatrix();
        const cv::Matx44f& frame1World = referencePose.GetInverseViewMatrix();
        const cv::Matx44f& frame2World = currentPose.GetInverseViewMatrix();

        // computed frame elements used for match filtering
        const auto fundamentalMatrixKcToKi = ComputeFundamentalMatrix(currentPose, currentFrame->GetUndistortedCalibration(), referencePose, referenceFrame->GetUndistortedCalibration());
        const auto fundamentalMatrixKiToKc = ComputeFundamentalMatrix(referencePose, referenceFrame->GetUndistortedCalibration(), currentPose, currentFrame->GetUndistortedCalibration());

        for (ptrdiff_t pointIterator = 0; pointIterator < frame1_points.size(); pointIterator++)
        {
//#define DGBEPI
#ifdef DBGEPI
            cv::Mat dbgImg2;
            cv::cvtColor( referenceFrame->GetDebugImage().clone(), dbgImg2, CV_GRAY2RGB);
            DebugRenderEpipolarLine(fundamentalMatrixKcToKi, currentFrame->GetKeyPoint(matches[pointIterator].trainIdx).pt, referenceFrame->GetKeyPoint(matches[pointIterator].queryIdx).pt, dbgImg2);

            cv::Mat dbgImg1;
            cv::cvtColor(currentFrame->GetDebugImage().clone(), dbgImg1, CV_GRAY2RGB);
            DebugRenderEpipolarLine(fundamentalMatrixKiToKc, referenceFrame->GetKeyPoint(matches[pointIterator].queryIdx).pt, currentFrame->GetKeyPoint(matches[pointIterator].trainIdx).pt, dbgImg1);
#endif

            // epipolar line test
            const float KcToKiEpipolarDistance = DistanceFromEpipolarLine(fundamentalMatrixKcToKi, currentFrame->GetKeyPoint(matches[pointIterator].trainIdx).pt, referenceFrame->GetKeyPoint(matches[pointIterator].queryIdx).pt);
            const float KiToKcEpipolarDistance = DistanceFromEpipolarLine(fundamentalMatrixKiToKc, referenceFrame->GetKeyPoint(matches[pointIterator].queryIdx).pt, currentFrame->GetKeyPoint(matches[pointIterator].trainIdx).pt);
            if (KiToKcEpipolarDistance + KcToKiEpipolarDistance > 2 * pixelMaxEpipolarDistance)
            {
                continue;
            }

            const cv::Point3f current3DPoint = TriangulatePointWorldSpace(frame1InvCam, frame1World, frame2InvCam, frame2World,
                frame1_points[pointIterator], frame2_points[pointIterator]);
         
            // Validate that the points projects in front of the camera
            if (!VerifyProjectInFront(referencePose.GetViewMatrix(), referenceFrame->GetUndistortedCalibration().GetCameraMatrix(), current3DPoint)
                || !VerifyProjectInFront(currentPose.GetViewMatrix(), currentFrame->GetUndistortedCalibration().GetCameraMatrix(), current3DPoint))
            {
                continue;
            }

            // Distance Ratio Test
            // To avoid creating points near the camera that will be culled later, we do a distance ratio test, where 
            // points that are near the keyframe, relative to the distance between the keyframes that see the point, are
            // not added.
            cv::Vec3f KiToTriangulatedPoint = current3DPoint - referencePose.GetWorldSpacePosition();
            float KiToTriangulatedPointDistance = sqrtf(KiToTriangulatedPoint.dot(KiToTriangulatedPoint));
            cv::Vec3f KiToKc = referencePose.GetWorldSpacePosition() - currentPose.GetWorldSpacePosition();
            float KiToKcDistance = sqrtf(KiToKc.dot(KiToKc));
            float distanceRatio = KiToTriangulatedPointDistance / KiToKcDistance;
            if (distanceRatio < minAcceptanceDistanceRatio)
            {
                continue;
            }

            // enforce the octaves to match
            if (referenceFrame->GetKeyPoint(matches[pointIterator].queryIdx).octave != currentFrame->GetKeyPoint(matches[pointIterator].trainIdx).octave)
            {
                continue;
            }

            // TODO enforce gridding?

            initial3DPoints.emplace_back(matches[pointIterator], current3DPoint);
        }
    }

    bool MapInitialization::InitializeWithFrames(
        gsl::span<const cv::DMatch> matches,
        const shared_ptr<const AnalyzedImage>& referenceFrame, //query
        const shared_ptr<const AnalyzedImage>& currentFrame,   //train
        InitializationData& initializationData,
        thread_memory memory)
    {
        SCOPE_TIMER(MapInitialization::InitializeWithFrames);

        // TODO try to refactor so that we use KeyframeBuilders throughout instead?
        mage::temp::vector<InitializationPose> initializationPoses = memory.stack_vector<InitializationPose>(3);

        std::vector<pair<cv::DMatch, cv::Point3f>> initial3DPoints;
        // ensure that enough matches have been accepted during the collection step
        if (matches.size() < (int)m_settings.MinFeatureMatches)
        {
            LogMessage<Tracing::TraceLevels::Verbose>(L"MapInitialization  Not enough accepted matches");
            return false;
        }

        vector<cv::Point2f> frame1_points;
        vector<cv::Point2f> frame2_points;
        CollectMatchPoints(referenceFrame, currentFrame, matches, frame1_points, frame2_points);

        Pose referencePose{};
        Pose currentPose{};

        vector<Pose> possiblePoses = FindPossiblePoses(frame1_points, frame2_points, referenceFrame->GetUndistortedCalibration(), currentFrame->GetUndistortedCalibration());
        DETERMINISTIC_CHECK(m_determinator, possiblePoses.begin(), possiblePoses.end());

        bool foundPose = FindCorrectPose(frame1_points, frame2_points, matches, referenceFrame->GetUndistortedCalibration(), currentFrame->GetUndistortedCalibration(), referencePose, possiblePoses, currentPose, initial3DPoints);
        DETERMINISTIC_CHECK(m_determinator, foundPose);

        if (!foundPose)
        {
            LogMessage<Tracing::TraceLevels::Verbose>(L"MapInitialization didn't find correct pose");
            return false;
        }

        assert(m_initializationFrames.size() >= 2);
        DETERMINISTIC_CHECK(m_determinator, initial3DPoints.data(), sizeof(initial3DPoints.front()) * initial3DPoints.size());

        // compute the associations with the resulting map points
        std::vector<PointAssociation> frontAssociations;
        std::vector<PointAssociation> backAssociations;
        frontAssociations.reserve(initial3DPoints.size());
        backAssociations.reserve(initial3DPoints.size());

        for (size_t iterator = 0; iterator < initial3DPoints.size(); iterator++)
        {
            frontAssociations.emplace_back(static_cast<size_t>(initial3DPoints[iterator].first.queryIdx), iterator);
            backAssociations.emplace_back(static_cast<size_t>(initial3DPoints[iterator].first.trainIdx), iterator);
        }

        initializationPoses.clear();
        initializationPoses.emplace_back(m_initializationFrames.front().Image, referencePose, frontAssociations);

        // add a placeholder for the third frame
        bool addThirdFrame = m_initializationFrames.size() > 2;
        if (addThirdFrame)
        {
            MatchedImage additionalImage = m_initializationFrames[m_initializationFrames.size() / 2];
            initializationPoses.emplace_back(additionalImage.Image, Pose(), std::vector<PointAssociation>{});
        }

        // add the back frame
        initializationPoses.emplace_back(m_initializationFrames.back().Image, currentPose, backAssociations);

        // create the initialization map
        {
            // create the mappoint proxies to link the keyframe builders to
            initializationData.Clear();

            std::transform(initial3DPoints.cbegin(), initial3DPoints.cend(), std::back_inserter(initializationData.MapPoints),
                [](const auto& point3D)
            {
                // MapInitialization map points are special and get treated as though they have been adjusted once already (which is technically true)
                return MapPointTrackingProxy::CreateNew(point3D.second, 1);
            });

            DETERMINISTIC_CHECK(m_determinator, initializationData.Frames.begin(), initializationData.Frames.end());

            // populate up keyframebuilders with the information we have created thus far
            for (size_t i = 0; i < initializationPoses.size(); i++)
            {
                const InitializationPose& initializationPose = initializationPoses[i];
                const std::shared_ptr<KeyframeBuilder>& keyframeBuilder = initializationData.AddKeyframeBuilder(initializationPose.Image, initializationPose.Pose);

                DETERMINISTIC_CHECK(m_determinator, *keyframeBuilder);

                // add all the point associations to the first and last frames
                if (i == 0 || i == initializationPoses.size() - 1)
                {
                    for (const auto& assoc : initializationPose.Associations)
                    {
                        keyframeBuilder->AddAssociation(initializationData.MapPoints[assoc.PointIndex3d], assoc.PointIndex2d);
                    }
                }

                DETERMINISTIC_CHECK(m_determinator, *keyframeBuilder);
            }

            DETERMINISTIC_CHECK(m_determinator, initializationData.Frames.begin(), initializationData.Frames.end());
        }

        return true;
    }

    void MapInitialization::BundleAdjustInitializationData(InitializationData& initializationData, mira::determinator& determinator, bool cullOutliers, const BundlerSettings& bundlerSettings, thread_memory memory)
    {
        // now bundle adjust the whole system to get a better positions and map point locations
        AdjustableData baData{};

        baData.Keyframes.reserve(initializationData.Frames.size());
        for (const auto& builder : initializationData.Frames)
        {
            baData.Keyframes.emplace_back(Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints>{*builder});

            builder->IterateAssociations([&](const MapPointTrackingProxy& mp, KeypointDescriptorIndex idx)
            {
                baData.MapPointAssociations.emplace_back(builder->GetAnalyzedImage()->GetKeyPoint(idx).pt, mp.GetId(), builder->GetId());
            });
        }

        // only the first keyframe is fixed, all others are allowed to float
        baData.Keyframes.front().SetFixed(true);

        std::swap(baData.MapPoints, initializationData.MapPoints);

        BundleAdjust ba{ baData, determinator, memory };
        ba.RunBundleAdjustment(
            BundleAdjust::NoOpScheduler{},
            bundlerSettings.HuberWidth,
            bundlerSettings.MaxOutlierError,
            bundlerSettings.MaxOutlierErrorScaleFactor,
            bundlerSettings.MinMeanSquareError,
            bundlerSettings.FixMapPoints,
            bundlerSettings.NumStepsPerRun,
            bundlerSettings.NumSteps,
            bundlerSettings.MinSteps,
            memory);

        auto outliers = ba.GetOutliers();

        DETERMINISTIC_CHECK(determinator, baData.Keyframes);
        DETERMINISTIC_CHECK(determinator, baData.MapPoints);
        DETERMINISTIC_CHECK(determinator, baData.MapPointAssociations);
        DETERMINISTIC_CHECK(determinator, outliers.data(), sizeof(outliers[0]) * outliers.size());

        std::swap(baData.MapPoints, initializationData.MapPoints);

        // update the keyframes
        for (size_t keyframeIterator = 0; keyframeIterator < baData.Keyframes.size(); keyframeIterator++)
        {
            // skip keyframes that were marked as fixed
            if (!baData.Keyframes[keyframeIterator].IsFixed())
            {
                initializationData.Frames[keyframeIterator]->SetPose(baData.Keyframes[keyframeIterator].GetPose());
            }

            // update the position of the mappoints in the keyframe builders by iterating over the associations
            initializationData.Frames[keyframeIterator]->IterateAssociations([&initializationData](MapPointTrackingProxy& mp, KeypointDescriptorIndex /*idx*/)
            {
                auto foundMapPoint = std::find_if(initializationData.MapPoints.begin(), initializationData.MapPoints.end(), [&mp](const MapPointTrackingProxy& adjustedMapPoint)
                {
                    return mp.GetId() == adjustedMapPoint.GetId();
                });
                assert(foundMapPoint != initializationData.MapPoints.end());

                mp.SetPosition(foundMapPoint->GetPosition());
            });
        }

        DETERMINISTIC_CHECK(determinator, initializationData.Frames.begin(), initializationData.Frames.end());

        if (cullOutliers)
        {

            // compute initial association counts for map points
            mage::temp::vector<std::pair<const Id<MapPoint>, int>> mapPointAssociationCounts = memory.stack_vector<std::pair<const Id<MapPoint>, int>>(initializationData.MapPoints.size());
            for (const MapPointTrackingProxy& proxy : initializationData.MapPoints)
            {
                mapPointAssociationCounts.emplace_back(proxy.GetId(), 0);
            };
            for (const auto& keyframe : initializationData.Frames)
            {
                std::vector<MapPointAssociations<MapPointTrackingProxy>::Association> associations;
                keyframe->GetAssociations(associations);
                for (const MapPointAssociations<MapPointTrackingProxy>::Association& assoc : associations)
                {
                    auto pair = std::find_if(mapPointAssociationCounts.begin(), mapPointAssociationCounts.end(), [assoc](auto& candidatePair) { return assoc.MapPoint.GetId() == candidatePair.first; });
                    pair->second++;
                }
            }

            DETERMINISTIC_CHECK(determinator, initializationData.Frames.begin(), initializationData.Frames.end());

            // cull the outlier associations
            for (const auto& outlierAssociation : outliers)
            {
                // find the associated keyframe
                auto keyframe = std::find_if(initializationData.Frames.begin(), initializationData.Frames.end(),
                    [outlierAssociation](auto& kb) { return kb->GetId() == outlierAssociation.second; });
                assert(keyframe != initializationData.Frames.end());

                // find the associated map point
                auto mapPoint = std::find_if(initializationData.MapPoints.cbegin(), initializationData.MapPoints.cend(),
                    [outlierAssociation](auto& mp) { return mp.GetId() == outlierAssociation.first; });
                assert(mapPoint != initializationData.MapPoints.end());

                keyframe->get()->RemoveAssociation(mapPoint->GetId());
                auto pair = std::find_if(mapPointAssociationCounts.begin(), mapPointAssociationCounts.end(), [mapPoint](auto& candidatePair) { return mapPoint->GetId() == candidatePair.first; });
                pair->second--;
            }

            DETERMINISTIC_CHECK(determinator, initializationData.Frames.begin(), initializationData.Frames.end());

            // now we need to go through and remove any map points which no longer have enough connections and disassociate them
            for (const auto& associationCount : mapPointAssociationCounts)
            {
                if (associationCount.second < 2)
                {
                    // remove this from any keyframes, and then remove the map point all together
                    for (auto& keyframe : initializationData.Frames)
                    {
                        keyframe->TryRemoveAssociation(associationCount.first);
                    }

                    // remove the element from the vector by swapping the back (order doesn't matter for us)
                    auto iter = std::find_if(initializationData.MapPoints.begin(), initializationData.MapPoints.end(), [associationCount](const MapPointTrackingProxy& proxy) { return associationCount.first == proxy.GetId(); });
                    assert(iter != initializationData.MapPoints.end());
                    std::swap(*iter, initializationData.MapPoints.back());
                    initializationData.MapPoints.pop_back();
                }
            }

            DETERMINISTIC_CHECK(determinator, initializationData.Frames.begin(), initializationData.Frames.end());
        }
    }

    bool MapInitialization::ValidateInitializationData(const InitializationData& initializationData,
        float maxPoseContributionZ,
        float amountBACanChangePose,
        size_t minMapPoints)
    {
        if (initializationData.MapPoints.size() < minMapPoints)
        {
            return false;
        }

        //TODO: check for reversed init after bundle adjustment

        // make sure that normalized the bundled system isn't too Z biassed
        const cv::Point3f& world = initializationData.Frames.back()->GetPose().GetWorldSpacePosition();
        const cv::Point3f normalizedWorld = Normalize(world);
        if (abs(normalizedWorld.z) > maxPoseContributionZ)
        {
            return false;
        }

        // enforce that the 'back' frame is close to initial scale (1.0)
        float scale = sqrt(world.dot(world));
        if ((amountBACanChangePose != 0) && ((scale < 1.0f / amountBACanChangePose) || (scale > 1.0f * amountBACanChangePose)))
        {
            return false;
        }

        // enforce that any extra frames are at a reasonable scale 2x initial scale
        // note that pose[0] should be identity, and pose[size-1] should be near to 1 as verified above
        for (size_t iterator = 1; iterator < initializationData.Frames.size() - 1; iterator++)
        {
            const cv::Point3f& position = initializationData.Frames[iterator]->GetPose().GetWorldSpacePosition();
            float norm = sqrt(position.dot(position));
            if (norm > 2.0f * scale)
            {
                return false;
            }
        }

        return true;
    }
}
