// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/iterators.h>
#include <arcana/math.h>
#include <arcana/propertybag.h>
#include <memory>

namespace mage
{
    enum class CameraIdentity
    {
        MONO,
        STEREO_1,
        STEREO_2
    };

    enum class FilterType
    {
        NONE = 0,
        FUSER3DOF = 1,
        FUSER6DOF = 2,
        SIMPLE6DOF = 3
    };

    enum class PosePriorMethod
    {
        MOTION_MODEL = 0,
        VISUAL_INERTIAL_FUSION = 1,
        VISUAL_INERTIAL_FUSION_WITH_3DOF = 2
    };

    // Settings for the feature matcher
    PROPERTYBAG(OrbMatcherSettings,
        PROPERTY(unsigned int, MaxHammingDistance, 30)                   // the maximum distance the features need to be from each other to be a match
        PROPERTY(unsigned int, MinHammingDifference, 1)                  // the minimum difference betwwen feature matches to ensure low ambiguity
    );

    PROPERTYBAG(BundleAdjustSettings,
        PROPERTY(unsigned int, NumSteps, 1)                          // number of iterations to run g2o
        PROPERTY(unsigned int, NumStepsPerRun, 1)                    // number of steps per iteration within g2o
        PROPERTY(unsigned int, MinSteps, 1)                          // min number of iteration within g2o for each keyframe
        PROPERTY(float, HuberWidth, 1.8f)                            // Huber width inside bundle adjust
        PROPERTY(float, HuberWidthScale, 0.95f)                      // Amount to scale Hube width when adding additional iterations
        PROPERTY(float, MaxOutlierError, 7.25f)                      // pixel error for reprojected to be called an outlier.
        PROPERTY(float, MaxOutlierErrorScaleFactor, 0.95f)           // each iteration will multiply the Max Outlier Error by this factor to give reducing width per iteration
        PROPERTY(float, MinMeanSquareError, 0.25f)                   // will stop iterating on local bundle adjust when this error is below this.
        PROPERTY(float, DistanceTetherWeight, 50.f)                  // the weight, in firkins, that is afforded to distance constraints on keyframes
        PROPERTY(float, LowConnectivityIterationsScale, 1.5f)        // If low amount of connectivity (initialization period) allow for extra bundling iterations to fill idle time in the schedule
    );

    PROPERTYBAG(NewMapPointsCreationSettings,
        PROPERTY(float, MinParallaxDegrees, 0.0238961594253207f)                    // Minimum angle, in degrees, between viewing positions for new map points creation.
        PROPERTY(float, MaxEpipolarError, 3.84385518580709f)                        // error threshold for considering whether a newly created mappoint would be on the epipolar line
        PROPERTY(float, MinAcceptedDistanceRatio, 2.0f)                             // Ratio of the distance to the newpoint and distance between keyframes.
        PROPERTY(float, MinKeyframeDistanceForCreatingMapPointsSquared, 0.0f)       // The minimum distance between keyframes use to triangulate new map points
        PROPERTY(float, MaxKeyframeAngleDegrees, 60.0f)                             // Maximum angle between newly found map points and associated keyframes to also match
        PROPERTY(float, NewMapPointsSearchRadius, 11.8816156f)                      // Search radius for matching map points in LocallyAssociateNewAssociations
        PROPERTY(unsigned int, MaxFramesForNewPointsCreation, 5)                     // The maximum number of frames to check for new points.
        NAMED_BAG_PROPERTY(OrbMatcherSettings, InitialMatcherSettings)
        NAMED_BAG_PROPERTY(OrbMatcherSettings, AssociateMatcherSettings)
    );

    PROPERTYBAG(GraphOptimizationSettings,
        PROPERTY(float, MaxOutlierError, 7.25f)                      // pixel error for reprojected to be called an outlier.
        PROPERTY(float, MaxOutlierErrorScaleFactor, 0.95f)           // each iteration will multiply the Max Outlier Error by this factor to give reducing width per iteration
        PROPERTY(unsigned int, NumSteps, 0)                           // number of iterations to run g2o
        PROPERTY(float, BundleAdjustmentHuberWidth, 0.372231848644798f)             // Huber width inside bundle adjust
    );

    PROPERTYBAG(CovisibilitySettings,
        PROPERTY(unsigned int, CovisMinThreshold, 15)           // min number of map points in common to gain an edge in covisibility
        PROPERTY(unsigned int, CovisLoopThreshold, 30)          // min number of map points in common to be considered a loop candidate
        PROPERTY(unsigned int, CovisEssentialThreshold, 100)    // min number of map points in common to be considered for the essential graph
        PROPERTY(unsigned int, UpperConnectionsForBA, 2000)     // Upper bound on connections to step up covis threshold for BA
        PROPERTY(unsigned int, LowerConnectionsForBA, 1500)     // Lower bound on connections to step down covis threshold for BA
        PROPERTY(unsigned int, CovisBaStepThreshold, 15)        // Amount to step covis when it is outside desired range
        PROPERTY(unsigned int, MaxSteps, 1)                     // max steps per new keyframe of covis for ba
    );

    PROPERTYBAG(KeyframeSettings,
        PROPERTY(unsigned int, KeyframeDecisionMinFrameCount, 60)           // number of frames that had to have passed since last keyframe before a new keyframe can be added (paper section V E)
        PROPERTY(unsigned int, KeyframeDecisionMinFrameCountReloc, 20)      // number of frames that had to have passed since last relocalization before a new keyframe can be added (paper section V E)
        PROPERTY(unsigned int, KeyframeDecisionMinTrackingPointCount, 25)   // number of map points that need to be tracked in the current frame to be considered for a keyframe (paper section V E). paper is unclear if points refers to map points or keypoints. believed to be map points
        PROPERTY(float, KeyframeDecisionMaxTrackingPointOverlap, 0.25f)     // the current frame tracks less than this percentage of points compared to the reference keyframe Kref (paper section V E)
        PROPERTY(float, KeyframeDecisionMaxTrackingPointMatches, 300)       // the maximum number of matched features to be allowed to be a keyframe
        PROPERTY(float, MappingMaxTrackingPointOverlap, 0.9f)               // Overlap percentage that keyframes must stay under.
        PROPERTY(unsigned int, MinimumKeyframeCovisibilityCount, 3)         // the minimum number of other other keyframes which must have a point visible in order for the point to be counted as a candidate for keyframe culling
        PROPERTY(float, MinFrameMoveToMinDepthRatio, 0.13f)                 // Distance user must move relative to closest point before a new keyframe is allowed.
    );

    // Settings for map initialization
    PROPERTYBAG(MonoMapInitializationSettings,
        PROPERTY(float, FundamentalTransferErrorThreshold, 1.1f)                // symmetric transfer error threshould for the fundamental matrix
        PROPERTY(unsigned int, MinFeatureMatches, 65)                           // minimum number of features needed to build an initial mapping
        PROPERTY(unsigned int, MinScoringInliers, 50)                           // minimum number of inliers for scoring to accept a set
        PROPERTY(float, MinInlierPercentage, 0.5f)                              // require a minimum percentage of points as inliers for 5-point scoring
        PROPERTY(unsigned int, MinInitialMapPoints, 40)                         // require this number of map points from the initial pair of keyframes
        PROPERTY(unsigned int, MinMapPoints, 60)                                // minimum number of map points needed for an initialization to pass
        PROPERTY(float, MinThirdFrameMatchPercentage, 0.5f)                     // minimum percentage of inlier matches required in the third frame after bundle adjustment
        PROPERTY(float, FeatureCovisibilityThreshold, 0.35f)                    // percentage of the frames which must contain a feature in order for it to be considered an initial map point
        PROPERTY(float, MaxParallax3dDistance, 500.0f)                          // maximum distance from camera for a 3d point
        PROPERTY(float, MaxParallax3dMedianDistance, 20.0f)                     // maximum median distance from camera for a 3d point
        PROPERTY(float, MinCandidatePoseDisimilarity, 0.3f)                     // necessary difference percentage from pose scores to consider the best pose unique enough from its next best neighbor
        PROPERTY(float, MaxPoseContributionZ, 0.66f)                            // indicates the maximum allowed move in the Z direction for initial pose estimation
        PROPERTY(unsigned int, BundleAdjustmentG2OSteps, 5)                     // number of steps we want to use with doing g2o bundle adjustment
        PROPERTY(float, BundleAdjustmentHuberWidth, 1.5f)                       // Huber width inside bundle adjust
        PROPERTY(unsigned int, RansacIterationsForModels, 90)                   // number of iterations to go through for ransac when computing homography and fundamental model
        PROPERTY(float, MaxEpipolarError, 3.5f)                                 // error threshold for considering whether a map points in map init
        PROPERTY(float, MaxOutlierError, 2.5f)                                  // pixel error for reprojected to be called an outlier.
        PROPERTY(float, AmountBACanChangePose, 1.65f)                           // the amount that bundle adjust can change the pose by (3x) before it is thrown out
        PROPERTY(float, MapInitializationNewPointsCreationMinDistance, 0.25f)   // the distance (in units of map scale)  required to consider frames for new points creation
        PROPERTY(unsigned int, MapInitFrameIntervalMilliseconds, 0)             // frame interval to investigate potential initialization pairs
        PROPERTY(unsigned int, MinInitializationIntervalMilliseconds, 150)      // minimum number of milliseconds passed between frames before attempting an initialization
        PROPERTY(unsigned int, MaxInitializationIntervalMilliseconds, 540)      // maximum number of milliseconds passed between frames before attempting an initialization
        PROPERTY(float, MinPixelSpread, 40.0f)                                  // minimum pixel distance between features in a ransac set
        PROPERTY(float, FinalBA_HuberWidth, 0.9f)                               // Huber width for final MapInitialization BA
        PROPERTY(float, FinalBA_MaxOutlierError, 4.0f)                          // Outlier error for final MapInitialization BA
        PROPERTY(float, FinalBA_MaxOutlierErrorScaleFactor, 0.75f)              // Error Scale factor for final MapInitialization BA
        PROPERTY(float, FinalBA_MinMeanSquareError, 0.0f)                       // Minimum error value to stop BA iterations for final MapInitialization
        PROPERTY(unsigned int, FinalBA_NumStepsPerRun, 5)                       // Number of BA steps per run for final MapInitialization BA
        PROPERTY(unsigned int, FinalBA_NumSteps, 15)                            // Total Number of BA steps for final MapInitialization BA
        PROPERTY(float, ExtraFrame_MaxOutlierError, 8.0f)                       // Maximum outlier error allowed when bundling additional keyframes
        PROPERTY(unsigned int, ExtraFrame_BundleAdjustmentSteps, 5)             // Number of BA steps for positioning extra frames for map init
        PROPERTY(float, ExtraFrame_HuberWidth, 4.0f)                            // Huber Width for BA of additional frames
        PROPERTY(float, ExtraFrame_SearchRadius, 40)                            // Search radius for finding features when adding additional frames
        //TODO convert BA settings into BundlerSettings abstraction to simplify?
        NAMED_BAG_PROPERTY(OrbMatcherSettings, FivePointMatchingSettings)
        NAMED_BAG_PROPERTY(OrbMatcherSettings, ExtraFrameMatchingSettings)
        BAG_PROPERTY(NewMapPointsCreationSettings)
    );

    PROPERTYBAG(StereoMapInitializationSettings,
        PROPERTY(unsigned int, MinInitMapPoints, 15)                              // minimum number of map points for a successful init
        PROPERTY(unsigned int, MinFeatureMatches, 40)
        PROPERTY(float, MaxOutlierError, 2.5f)
        PROPERTY(float, MaxEpipolarError, 5.5f)                                 // error threshold for considering whether a map points in map init
        PROPERTY(float, MinAcceptedDistanceRatio, 2.0f)                         // Ratio of the distance to the newpoint and distance between keyframes.
        PROPERTY(float, InitializationTetherStrength, 50.0f)                    // strength to assign the tether for the initialization stereo pair
        PROPERTY(float, MaxPoseContributionZ, 0.10f)                            // indicates the maximum allowed move in the Z direction for initial pose estimation
        PROPERTY(float, AmountBACanChangePose, 1.65f)                           // the amount that bundle adjust can change the pose by before it is thrown out
        PROPERTY(float, MaxDepthMeters, 2.3f)                                   // temporary setting that describes the distance beyond which depth is unacceptably uncertain.
        BAG_PROPERTY(OrbMatcherSettings)
        BAG_PROPERTY(BundleAdjustSettings)
    );

    
    // Settings for the Orb Extractor
    PROPERTYBAG(FeatureExtractorSettings,
        PROPERTY(unsigned int, NumFeatures, 440)                    // total number of features to detect
        PROPERTY(float, ScaleFactor, 1.5f)                          // the scale factor for the image pyramid
        PROPERTY(unsigned int, GaussianKernelSize, 7)               // must be positive and odd.  Values <= 1 indicate no blurring
        PROPERTY(unsigned int, NumLevels, 1)                        // the depth of the image pyramid
        PROPERTY(int, FastThreshold, 4)                             // the threshold for fast feature detection
        PROPERTY(unsigned int, PatchSize, 15)                       // size of the patch used by the oriented BRIEF descriptor, can't be larger than 2 * imageBorder + 1
        PROPERTY(bool, UseOrientation, false)                       // True will use ORB features false BRIEF.
        PROPERTY(float, FeatureFactor, 1.5f)                        // factor of numFeatures will be extracted and suppressed to 440, higher factor will increase possibility to distribute features better but also allow weak features.
        PROPERTY(float, FeatureStrength, 0.9f)                      // We will not use features weaker than than weakest feature (among m_nfeatures) times this constant.
        PROPERTY(int, StrongResponse, 20)                           // What's considered to be a "strong feature" i.e. if we can distribute features spatially well we are happy. This works together with robustnessFactors below.
        PROPERTY(float, MinRobustnessFactor, 1.1f)                  // The lower this factor is the more we try to distribute features spatially.
        PROPERTY(float, MaxRobustnessFactor, 2.0)                   // The higher this factor is the more we prefere features with strongResponse vs spatially distribution.
        PROPERTY(int, NumCellsX, 32)
        PROPERTY(int, NumCellsY, 32)
        float GetImageBorder() const { return PatchSize / 2.0f; }   // compute the ImageBorder from the patch size
    );

    // Settings for pose estimation
    PROPERTYBAG(PoseEstimationSettings,
        PROPERTY(float, SearchRadius, 12.0f)                           // the radius within to search initially for matches
        PROPERTY(float, WiderSearchRadius, 24.0f)                     // the factor with which to increase the search radius during pose estimation
        PROPERTY(float, ExtraWiderSearchRadius, 36.0f)                     // the factor with which to increase the search radius during pose estimation
        PROPERTY(unsigned int, FeatureMatchThreshold, 20)           // min number of features to match
        PROPERTY(float, FeatureSmallMatchRatioThreshold, 0.333780871615353f)     // min ratio for feature that were found when doing small radius search, if lower force wider search
        PROPERTY(unsigned int, MinMapPointRefinementCount, 0)       // minimum number of adjustments a mappoint must endure before being used during pose estimation
        BAG_PROPERTY(OrbMatcherSettings)
    );

    PROPERTYBAG(TrackLocalMapSettings,
        PROPERTY(float, MinDegreesBetweenCurrentViewAndMapPointView, 60)        //angle in degrees between the current viewing ray and the map point mean viewing direction cannot be less than this
        PROPERTY(unsigned int, BundleAdjustmentG2OSteps, 4)                    // number of steps we want to use with doing g2o bundle adjustment
        PROPERTY(float, BundleAdjustmentHuberWidth, 0.9f)                      // Huber width inside bundle adjust
        PROPERTY(unsigned int, InitialPoseEstimateBundleAdjustmentSteps, 3)     // number of steps we want to use with doing g2o bundle adjustment for the initial pose estimate
        PROPERTY(float, InitialPoseEstimateBundleAdjustmentHuberWidth, 4.0f)     // Huber width inside bundle adjust
        PROPERTY(float, RecentMapPointPctSuccess, 0.137686914508039f)                        // the minimum ratio of times a map point must be found vs. expected by the tracker to avoid culling while it is under recent map point scrutiny VI- Local Mapping B - Recent Map Points Culling - 1)
        PROPERTY(float, MatchSearchRadius, 8.0f)                                 // the radius within which to search for a match in the unassociated array
        PROPERTY(float, MaxOutlierError, 4.5f)                                  // pixel error for reprojected to be called an outlier.
        PROPERTY(float, MaxOutlierErrorPoseEstimation, 6.0f)                    // pixel error for outliers after first pose estimation BA
        PROPERTY(bool, UnassociateOutliers, true)                         // toggle to control if TrackLocalMap bundleAdjust outliers are unassociated as MapPoints
        PROPERTY(unsigned int, TrackingLostCountUntilReloc, 3)                    // The number of tracking frames to be lost before trying to fall back to reloc
        PROPERTY(unsigned int, MinMapPointRefinementCount, 0)       // minimum number of adjustments a mappoint must endure before being used during pose estimation
        PROPERTY(unsigned int, MinTrackedFeatureCount, 20)          // minimum number of features to consider this frame tracked
        BAG_PROPERTY(OrbMatcherSettings)
    );

    PROPERTYBAG(LoopClosureSettings,
        PROPERTY(bool, EnableLoopClosure, false)                               // Whether or not to even attempt loop closure.
        PROPERTY(unsigned int, MaxMapPoints, 200)                              // Max number of points to reproject when making a new keyframe.
        PROPERTY(float, MatchSearchRadius, 18.f)                               // Max search radius in loop closure.
        PROPERTY(unsigned int, MinKeyframe, 10)                                // Min number of key frames before starting to try loop closure.
        PROPERTY(unsigned int, MinClusterSize, 3)                              // Min number of keyframes in a cluster for the loop closure to be accepted.
        PROPERTY(unsigned int, MinFeatureMatches, 0)                           // Minimum feature matches for the loop closure pose estimation
        BAG_PROPERTY(BundleAdjustSettings)                                     // The settings for the final global bundle adjust performed after loop closure.
        NAMED_BAG_PROPERTY(OrbMatcherSettings, CheapLoopClosureMatchingSettings)                                       // Matching settings for Cheap Loop Closure
        NAMED_BAG_PROPERTY(OrbMatcherSettings, MapMergeMatchingSettings)
    );

    PROPERTYBAG(PoseHistorySettings,
        PROPERTY(size_t, InitalInterpolationConnections, 4)                     // number of keyframes to connect each tracking frame to when first created
        PROPERTY(size_t, MaxInterpolationConnections, 1)                        // max number of keyframes to connect each tracking frame to using each new keyframe as created.
        PROPERTY(size_t, PoseHistoryInitialSize, 10000)                         // initial capacity of the pose history (10000 ~ 5.5 minutes)
        PROPERTY(size_t, KeyframeHistoryInitialSize, 1000)                      // initial capacity of the keyframe history used to interpolate poses
    );

    PROPERTYBAG(BoundingDepthSettings,
        PROPERTY(float, RegionOfInterestMinX, 0.1f)             // smallest X pixel value within the region of interest
        PROPERTY(float, RegionOfInterestMinY, 0.1f)             // smallest Y pixel value within the region of interest
        PROPERTY(float, RegionOfInterestMaxX, 0.9f)            // largest X pixel value within the region of interest
        PROPERTY(float, RegionOfInterestMaxY, 0.9f)            // largest Y pixel value within the region of interest
        PROPERTY(float, NearDepthSoftness, 0.f)                  // Sensitivity of near depth filter (higher value deviates from "true" nearest observation)
        PROPERTY(float, FarDepthSoftness, 0.f)                   // Sensitivity of far depth filter (higher value deviates from "true" farthest observation)
    );

    PROPERTYBAG(BagOfWordsSettings,
        PROPERTY(float, QualifyingCandidateScore, 0.75f)             // The fraction of the maximum "matching" score for a given query that must be exceeded for a result to be considered a candidate
        PROPERTY(bool, UseDirectIndex, true)                        // Whether or not the bag of words should build a direct index, mapping from descriptors to word levels (see TemplatedDatabase.h)
        PROPERTY(unsigned int, DirectIndexLevels, 4)                // Levels to go up the vocabulary tree to select the node id to store in the direct index when adding images (see TemplatedDatabase.h)
        PROPERTY(unsigned int, TrainingFrames, 15)                  // Number of frames used for training the vocabulary tree
        PROPERTY(unsigned int, TrainingTreeLevels, 2)               // Number of levels below the root node of the online vocabulary tree
        PROPERTY(unsigned int, TrainingTreeBranchingFactor, 6)      // Online Vocabulary tree 's branching factor
        PROPERTY(unsigned int, MaxTrainingIteration, 12)             // The max number of Kmean or Kmediod iteration while training the tree
        PROPERTY(unsigned int, MinTrainingSize, 1000)               // The minimum number of descriptors in the train set.
    );

    PROPERTYBAG(RelocalizationSettings,
        PROPERTY(unsigned int, MinBruteForceCorrespondences, 20)
        PROPERTY(unsigned int, MinRadiusMatchCorrespondences, 15)
        PROPERTY(unsigned int, MinMapPoints, 10)
        PROPERTY(float, RansacInliersPctRequired, 0.4f)
        PROPERTY(float, BundleAdjustInliersPctRequired, 0.4f)
        PROPERTY(float, RansacConfidence, 0.6f)
        PROPERTY(size_t, RoundRobinIterations, 5)
        PROPERTY(unsigned int, RansacIterations, 2)
        PROPERTY(unsigned int, BundleAdjustIterations, 10)
        PROPERTY(float, SearchRadius, 20)
        PROPERTY(float, MaxBundleAdjustReprojectionError, 8)
        PROPERTY(float, MaxBundlePnPReprojectionError, 8)
        BAG_PROPERTY(OrbMatcherSettings)
    );

    // mapping settings
    PROPERTYBAG(MappingSettings,
        PROPERTY(unsigned int, MaxRelocQueryResults, 4)                          // The maximum number of candidate keyframes to be returned by the bag of words for a single query
        PROPERTY(unsigned int, MaxPendingKeyframes, 4)                           // The maximum number of keyframes the map can fall behind before it starts dropping them.
        PROPERTY(unsigned int, MaxLoopClosureQueryResults, 1000)
        PROPERTY(unsigned int, MinNumKeyframesForMapPointCulling, 3)              // the minimum number of keyframes a mappoint must be seen in to avoid being culled
        PROPERTY(bool, UseCheapLoopClosure, true)                               // Toggles whether or not to have "cheap loop closure" enabled.
        PROPERTY(bool, PersistLambda, true)                                     // Toggles whether or not the lambda value for LM bundle adjustment should persist between local BAs.
        PROPERTY(float, MinLambda, 0.001f)                                      // Sets the minimum lambda value that will be persisted across local bundle adjusts.
        BAG_PROPERTY(NewMapPointsCreationSettings)
    );

    PROPERTYBAG(PosePriorSettings,
        PROPERTY(PosePriorMethod, PosePrior, PosePriorMethod::MOTION_MODEL)     // The technique to use for getting pose priors
        PROPERTY(bool, AssumeIMUAndCameraAreAtSamePosition, false)              // This removes the translation portion of the extrinsics and just assumes the two are colocated.
    );

    PROPERTYBAG(RuntimeSettings,
        PROPERTY(unsigned int, TrackingReadsPerLoopDetection, 2)                // Number of tracking read actions to take before the loop detection step.
        PROPERTY(unsigned int, TrackingReadsPerLoopClosure, 30)                 // Number of tracking read actions to take in parallel to the loop closure bundle adjust.
        BAG_PROPERTY(PosePriorSettings)
    );

    // the runtime settings related to inertial tracking
    PROPERTYBAG(FuserSettings,
        PROPERTY(bool, UseFuser, true)                  // enable the fuser which will attempt to combine IMU data with MAGE's visual updates
        PROPERTY(bool, ReturnFuserOutput, false)         // use the fuser's output as MAGE's answer to pose
        PROPERTY(bool, ApplyVisualUpdate, true)         // use mage's visual estimate to help the fuser
        PROPERTY(float, StdDevPoseError, 0.004f)        // the standard deviation of mage's average pose error (as reported by the scorer)
        PROPERTY(bool, DropMagSamples, true)            // don't allow magnetometer packets to be added to the sample queue (should be false in the recording tracker)
        PROPERTY(int, DeltaPoseRateMS, 66)              // to improve signal to noise ratio we apply delta pose updates at slower than frame rate
        PROPERTY(int,   MinDeltaPoseRateMS, 65)         // the minimum amount of time passed that should be considered a valid delta pose period (may change to affect signal to noise ratio)
        PROPERTY(int,   MaxDeltaPoseRateMS, 129)        // the maximum amount of time passed that should be considered a valid delta pose period (may change to affect signal to noise ratio)
        PROPERTY(FilterType, FilterType, FilterType::FUSER3DOF) // use the 6-dof mode or 3dof or simple6dof
        BAG_PROPERTY(OrbMatcherSettings)
    );

    // Volume of interest settings
    PROPERTYBAG(VolumeOfInterestSettings,
        PROPERTY(float, Threshold, 0.5f)                  // Numeric cutoff threshold for culling the volume of interest
        PROPERTY(int, Iterations, 3)                      // Number of times increasing-res bounding box refinement iterations, which will improve resolution in the final confidence blob
        PROPERTY(int, VoxelCountFloor, 16000)             // Minimum voxel resolution to compute the volume of interest
        PROPERTY(float, AwayProminence, 1.2f)             // Protrusion of the teardrop in the direction away from the camera, in units of (FarDepth - NearDepth)
        PROPERTY(float, TowardProminence, 0.1f)           // Protrusion of the teardrop in the direction toward the camera, in units of (FarDepth - NearDepth)
        PROPERTY(float, SideProminence, 1.f)              // Protrusion of the teardrop orthogonal to the camera's view direction, in units of the distance at that depth from the view vector to the nearest edge of the frame
        PROPERTY(float, KernelAngleXRads, mira::deg2rad(60.f)) // Angle in the appropriate axis of the "region of interest," which guides blob dimensions orthogonal to view direction.
        PROPERTY(float, KernelAngleYRads, mira::deg2rad(40.f)) // Angle in the appropriate axis of the "region of interest," which guides blob dimensions orthogonal to view direction.
                                                          //     TODO: Remove "KernelAngle" settings if/when FOV becomes inherently available from elsewhere in the program
                                                          //           (such as from PoseHistory).  Because it is conceivable for the relevant FOV to change from frame to
                                                          //           frame, it should be represented on a frame-by-frame basis; when that becomes available, this setting,
                                                          //           at least in its current form, will no longer be relevant.
        PROPERTY(float, KernelPitchRads, 0.f)             //
        PROPERTY(float, KernelRollRads, 0.f)              //
        PROPERTY(float, KernelYawRads, mira::deg2rad(5.f)) //
        PROPERTY(float, KernelDepthModifier, 1.f)         //
    );

    PROPERTYBAG(PerCameraSettings,
        BAG_PROPERTY(FeatureExtractorSettings)
        PROPERTY(unsigned int, NewPointGridWidth, 4)                            // grid width, see NewPointMaxGridCount
        PROPERTY(unsigned int, NewPointGridHeight, 3)                           // grid height, see NewPointMaxGridCount
        PROPERTY(unsigned int, NewPointMaxGridCount, 6)                         // stop generating new point points in this grid if we hit this count.
        PROPERTY(bool, UndistortImagePixels, false)                             // if true, will modify image to contain undistorted image pixels, if not, only keypoints will be undistorted
        PROPERTY(unsigned int, KeyframeDecisionGridWidth, 8)                    // A grid of the connected keypoints on a frame   
        PROPERTY(unsigned int, KeyframeDecisionGridHeight, 5)
        PROPERTY(unsigned int, KeyframeDecisionMinMapPointsPerGridCell, 2)  //the minimum number of map points a non-empty grid cell should have
        PROPERTY(float, KeyframeDecisionAllowedEmptyCellPercentage, 0.4f)   //If the percentage of empty cells in the grid is less than or equal to KeyframeDecisionAllowedEmptyCellPercentage, current frame may not be considered as a keyframe
    );

    PROPERTYBAG(StereoSettings,
        PROPERTY(bool, UseStereoInit, false)
        PROPERTY(CameraIdentity, PrimaryTrackingCamera, CameraIdentity::STEREO_2)   // what is the frame to track on if two frames are present
        NAMED_BAG_PROPERTY(PerCameraSettings, Camera1)
        NAMED_BAG_PROPERTY(PerCameraSettings, Camera2)
        BAG_PROPERTY(StereoMapInitializationSettings)
    );

    PROPERTYBAG(MonoSettings,
        NAMED_BAG_PROPERTY(PerCameraSettings, MonoCamera)
        BAG_PROPERTY(MonoMapInitializationSettings)
    );

    PROPERTYBAG(Metadata,
        PROPERTY(bool, LoadedFromFile, false)
        PROPERTY(unsigned int, TrackingWidth, 320)           //Resolution (width) of the main tracking camera that this settings file is tuned for
    );

    // Settings for the whole app, a container for all the other settings containers
    PROPERTYBAG(MageSlamSettings,
        BAG_PROPERTY(Metadata)
        BAG_PROPERTY(BundleAdjustSettings)          //TODO move into MappingSettings
        BAG_PROPERTY(GraphOptimizationSettings)
        BAG_PROPERTY(CovisibilitySettings)
        BAG_PROPERTY(KeyframeSettings)
        BAG_PROPERTY(PoseEstimationSettings)
        BAG_PROPERTY(RelocalizationSettings)
        BAG_PROPERTY(BagOfWordsSettings)
        BAG_PROPERTY(TrackLocalMapSettings)
        BAG_PROPERTY(PoseHistorySettings)
        BAG_PROPERTY(BoundingDepthSettings)
        BAG_PROPERTY(MappingSettings)
        BAG_PROPERTY(RuntimeSettings)
        BAG_PROPERTY(FuserSettings)
        BAG_PROPERTY(LoopClosureSettings)
        BAG_PROPERTY(VolumeOfInterestSettings)
        BAG_PROPERTY(StereoSettings)
        BAG_PROPERTY(MonoSettings)
    );

    const unsigned int INIT_NUM_KEYFRAMES = 100;                    // anywhere we preallocate room for keyframes, this is a good starting number. not an upper bound
    const unsigned int INIT_NUM_MAPPOINTS = 1000;                   // anywhere we preallocate room for mappoints, this is a good starting number. not an upper bound
    const unsigned int INIT_NUM_FEATUREPOINTS = 1000;               // anywhere we preallocate room for feature points in images, this is a good starting number. not an upper bound

    inline const PerCameraSettings& GetSettingsForCamera(const MageSlamSettings& settings, CameraIdentity cameraIdentity)
    {
        switch (cameraIdentity)
        {
        case mage::CameraIdentity::MONO:
            return settings.MonoSettings.MonoCamera;
        case mage::CameraIdentity::STEREO_1:
            return settings.StereoSettings.Camera1;
        case  mage::CameraIdentity::STEREO_2:
            return settings.StereoSettings.Camera2;
        default:
            assert(false && "Unhandled CameraIdentity enum");
            return settings.MonoSettings.MonoCamera;
        }
    }
}
