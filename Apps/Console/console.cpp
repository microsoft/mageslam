// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include <MageSlam.h>

#include <opencv2/opencv.hpp>

#include <iostream>

namespace
{
    constexpr auto INPUT_MP4 = "C:\\scratch\\video.mp4";
    constexpr auto OUTPUT_CSV = "C:\\scratch\\mage_output.csv";

    void ExportFossilCsv(const mage::MAGESlam::FossilizedMap& fossil, gsl::span<mage::FrameId> frameIds)
    {
        auto frames = fossil.GetTrackingResultsForFrames(frameIds);

        std::ofstream file{ OUTPUT_CSV };
        auto fileHandle = gsl::finally([&file]() { file.close(); });

        for (const auto& frame : frames)
        {
            if (frame)
            {
                file << "\"true\",";

                const auto& pose = frame->Tracking.Pose;
                file <<
                    pose.M11 << "," <<
                    pose.M12 << "," <<
                    pose.M13 << "," <<
                    pose.M14 << "," <<
                    pose.M21 << "," <<
                    pose.M22 << "," <<
                    pose.M23 << "," <<
                    pose.M24 << "," <<
                    pose.M31 << "," <<
                    pose.M32 << "," <<
                    pose.M33 << "," <<
                    pose.M34 << "," <<
                    pose.M41 << "," <<
                    pose.M42 << "," <<
                    pose.M43 << "," <<
                    pose.M44;
            }
            else
            {
                file << "\"false\",1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1";
            }

            file << std::endl;
        }
    }
}

void ProcessFrames(const char* videoFileName, std::function<void(const cv::Mat& frame)> processFunction)
{
    cv::namedWindow("Video", CV_WINDOW_AUTOSIZE);

    cv::VideoCapture vid(videoFileName);
    if (!vid.isOpened())
    {
        return;
    }

    cv::Mat frame{};
    cv::Mat grayscale{};
    cv::Mat trackingImage{};
    const cv::Size TRACKING_SIZE{ 320, 180 };
    for (size_t frameIdx = 0; vid.read(frame); frameIdx++)
    {
        auto type = frame.type();
        auto foo = CV_8UC3;

        cv::cvtColor(frame, grayscale, cv::COLOR_RGB2GRAY);

        cv::resize(grayscale, trackingImage, TRACKING_SIZE);

        cv::imshow("Video", trackingImage);
        processFunction(trackingImage);

        switch (cv::waitKey(1))
        {
        case 27:
            return;
        }
    }
}

void SetDefaultSettings(mage::MageSlamSettings& settings)
{
    settings.FuserSettings.UseFuser = false;

    // {
    //   "MageSlamSettings": {
    //     "Metadata": {
    //       "LoadedFromFile": true,
    settings.Metadata.LoadedFromFile = true;
    //       "TrackingWidth": 320
    settings.Metadata.TrackingWidth = 320;
    //     },
    //     "BundleAdjustSettings": {
    //       "MaxOutlierError": 3.0,
    settings.BundleAdjustSettings.MaxOutlierError = 3.f;
    //       "HuberWidth": 0.9
    settings.BundleAdjustSettings.HuberWidth = 0.9f;
    //     },
    //     "GraphOptimizationSettings": {
    //       "MaxOutlierError": 3.5
    settings.GraphOptimizationSettings.MaxOutlierError = 3.5f;
    //     },
    //     "LoopClosureSettings": {
    //       "BundleAdjustSettings": {
    //         "MinSteps": 25,
    settings.LoopClosureSettings.BundleAdjustSettings.MinSteps = 25;
    //         "NumSteps": 25,
    settings.LoopClosureSettings.BundleAdjustSettings.NumSteps = 25;
    //         "NumStepsPerRun": 25,
    settings.LoopClosureSettings.BundleAdjustSettings.NumStepsPerRun = 25;
    //         "HuberWidth": 0.372231848644798,
    settings.LoopClosureSettings.BundleAdjustSettings.HuberWidth = 0.372231f;
    //         "MaxOutlierError": 7.25
    settings.LoopClosureSettings.BundleAdjustSettings.MaxOutlierError = 7.25f;
    //       },
    //       "CheapLoopClosureMatchingSettings": {
    //         "MaxHammingDistance": 35,
    settings.LoopClosureSettings.CheapLoopClosureMatchingSettings.MaxHammingDistance = 35;
    //         "MinHammingDifference": 1
    settings.LoopClosureSettings.CheapLoopClosureMatchingSettings.MinHammingDifference = 1;
    //       },
    //       "MapMergeMatchingSettings": {
    //         "MaxHammingDistance": 20,
    settings.LoopClosureSettings.MapMergeMatchingSettings.MaxHammingDistance = 20;
    //         "MinHammingDifference": 1
    settings.LoopClosureSettings.MapMergeMatchingSettings.MinHammingDifference = 1;
    //       }
    //     },
    //     "KeyframeSettings": {
    //       "KeyframeDecisionMaxTrackingPointOverlap": 0.5
    settings.KeyframeSettings.KeyframeDecisionMaxTrackingPointOverlap = 0.5f;
    //     },
    //     "PoseEstimationSettings": {
    //       "MinMapPointRefinementCount": 1,
    settings.PoseEstimationSettings.MinMapPointRefinementCount = 1;
    //       "OrbMatcherSettings": {
    //         "MaxHammingDistance": 30,
    settings.PoseEstimationSettings.OrbMatcherSettings.MaxHammingDistance = 30;
    //         "MinHammingDifference": 1
    settings.PoseEstimationSettings.OrbMatcherSettings.MinHammingDifference = 1;
    //       }
    //     },
    //     "RelocalizationSettings": {
    //       "OrbMatcherSettings": {
    //         "MaxHammingDistance": 40,
    settings.RelocalizationSettings.OrbMatcherSettings.MaxHammingDistance = 40;
    //         "MinHammingDifference": 1
    settings.RelocalizationSettings.OrbMatcherSettings.MinHammingDifference = 1;
    //       }
    //     },
    //     "CovisibilitySettings": {
    //       "CovisMinThreshold": 10
    settings.CovisibilitySettings.CovisMinThreshold = 10;
    //     },
    //     "TrackLocalMapSettings": {
    //       "MaxOutlierError": 2.25,
    settings.TrackLocalMapSettings.MaxOutlierError = 2.25f;
    //       "MaxOutlierErrorPoseEstimation": 4,
    settings.TrackLocalMapSettings.MaxOutlierErrorPoseEstimation = 4.f;
    //       "MatchSearchRadius": 4,
    settings.TrackLocalMapSettings.MatchSearchRadius = 4.f;
    //       "InitialPoseEstimateBundleAdjustmentHuberWidth": 3.25,
    settings.TrackLocalMapSettings.InitialPoseEstimateBundleAdjustmentHuberWidth = 3.25f;
    //       "MinMapPointRefinementCount": 1,
    settings.TrackLocalMapSettings.MinMapPointRefinementCount = 1;
    //       "RecentMapPointPctSuccess": 0.25,
    settings.TrackLocalMapSettings.RecentMapPointPctSuccess = 0.25f;
    //       "OrbMatcherSettings": {
    //         "MaxHammingDistance": 35,
    settings.TrackLocalMapSettings.OrbMatcherSettings.MaxHammingDistance = 35;
    //         "MinHammingDifference": 1
    settings.TrackLocalMapSettings.OrbMatcherSettings.MinHammingDifference = 1;
    //       }
    //     },
    //     "PoseHistorySettings": {
    //       "InitialInterpolationConnections": 4,
    settings.PoseHistorySettings.InitalInterpolationConnections = 4;
    //       "MaxInterpolationConnections": 6
    settings.PoseHistorySettings.MaxInterpolationConnections = 6;
    //     },
    //     "MappingSettings": {
    //       "NewMapPointsCreationSettings": {
    //         "MaxEpipolarError": 5.5,
    settings.MappingSettings.NewMapPointsCreationSettings.MaxEpipolarError = 5.5f;
    //         "NewMapPointsSearchRadius": 11,
    settings.MappingSettings.NewMapPointsCreationSettings.NewMapPointsSearchRadius = 11.f;
    //         "MinParallaxDegrees": 0.25,
    settings.MappingSettings.NewMapPointsCreationSettings.MinParallaxDegrees = 0.25f;
    //         "MinKeyframeDistanceForCreatingMapPointsSquared": 0.25,
    settings.MappingSettings.NewMapPointsCreationSettings.MinKeyframeDistanceForCreatingMapPointsSquared = 0.25f;
    //         "InitialMatcherSettings": {
    //           "MaxHammingDistance": 25,
    settings.MappingSettings.NewMapPointsCreationSettings.InitialMatcherSettings.MaxHammingDistance = 25;
    //           "MinHammingDifference": 1
    settings.MappingSettings.NewMapPointsCreationSettings.InitialMatcherSettings.MinHammingDifference = 1;
    //         },
    //         "AssociateMatcherSettings": {
    //           "MaxHammingDistance": 35,
    settings.MappingSettings.NewMapPointsCreationSettings.AssociateMatcherSettings.MaxHammingDistance = 35;
    //           "MinHammingDifference": 1
    settings.MappingSettings.NewMapPointsCreationSettings.AssociateMatcherSettings.MinHammingDifference = 1;
    //         }
    //       }
    //     },
    //     "RuntimeSettings": {
    //       "TrackingReadsPerLoopDetection": 0
    settings.RuntimeSettings.TrackingReadsPerLoopClosure = 0;
    //     },
    //     "MonoSettings": {
    //       "MonoCamera": {
    //         "KeyframeDecisionAllowedEmptyCellPercentage": 0.6,
    settings.MonoSettings.MonoCamera.KeyframeDecisionAllowedEmptyCellPercentage = 0.6f;
    //         "FeatureExtractorSettings": {
    //           "NumFeatures": 440,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.NumFeatures = 440;
    //           "ScaleFactor": 1.5,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.ScaleFactor = 1.5f;
    //           "NumLevels": 1,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.NumLevels = 1;
    //           "FastThreshold": 4,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.FastThreshold = 4;
    //           "PatchSize": 15,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.PatchSize = 15;
    //           "FeatureFactor": 1.5,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.FeatureFactor = 1.5f;
    //           "StrongResponse": 20,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.StrongResponse = 20;
    //           "MinRobustnessFactor": 1.1,
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.MinRobustnessFactor = 1.1f;
    //           "MaxRobustnessFactor": 2.2
    settings.MonoSettings.MonoCamera.FeatureExtractorSettings.MaxRobustnessFactor = 2.2f;
    //         }
    //       },
    //       "MonoMapInitializationSettings": {
    //         "MinInlierPercentage": 0.65,
    settings.MonoSettings.MonoMapInitializationSettings.MinInlierPercentage = 0.65f;
    //         "MinInitialMapPoints": 40,
    settings.MonoSettings.MonoMapInitializationSettings.MinInitialMapPoints = 40;
    //         "FeatureCovisibilityThreshold": 0.35,
    settings.MonoSettings.MonoMapInitializationSettings.FeatureCovisibilityThreshold = 0.35f;
    //         "MaxInitializationIntervalMilliseconds": 330,
    settings.MonoSettings.MonoMapInitializationSettings.MaxInitializationIntervalMilliseconds = 330;
    //         "FinalBA_HuberWidth": 0.75,
    settings.MonoSettings.MonoMapInitializationSettings.FinalBA_HuberWidth = 0.75f;
    //         "FivePointMatchingSettings": {
    //           "MaxHammingDistance": 30,
    settings.MonoSettings.MonoMapInitializationSettings.FivePointMatchingSettings.MaxHammingDistance = 30;
    //           "MinHammingDifference": 1
    settings.MonoSettings.MonoMapInitializationSettings.FivePointMatchingSettings.MinHammingDifference = 1;
    //         },
    //         "ExtraFrameMatchingSettings": {
    //           "MaxHammingDistance": 30,
    settings.MonoSettings.MonoMapInitializationSettings.ExtraFrameMatchingSettings.MaxHammingDistance = 30;
    //           "MinHammingDifference": 1
    settings.MonoSettings.MonoMapInitializationSettings.ExtraFrameMatchingSettings.MinHammingDifference = 1;
    //         },
    //         "NewMapPointsCreationSettings": {
    //           "MaxEpipolarError": 2.0,
    settings.MonoSettings.MonoMapInitializationSettings.NewMapPointsCreationSettings.MaxEpipolarError = 2.f;
    //           "NewMapPointsSearchRadius": 7.0,
    settings.MonoSettings.MonoMapInitializationSettings.NewMapPointsCreationSettings.NewMapPointsSearchRadius = 7.f;
    //           "InitialMatcherSettings": {
    //             "MaxHammingDistance": 30,
    settings.MonoSettings.MonoMapInitializationSettings.NewMapPointsCreationSettings.InitialMatcherSettings.MaxHammingDistance = 30;
    //             "MinHammingDifference": 1
    settings.MonoSettings.MonoMapInitializationSettings.NewMapPointsCreationSettings.InitialMatcherSettings.MinHammingDifference = 1;
    //           },
    //           "AssociateMatcherSettings": {
    //             "MaxHammingDistance": 35,
    settings.MonoSettings.MonoMapInitializationSettings.NewMapPointsCreationSettings.AssociateMatcherSettings.MaxHammingDistance = 35;
    //             "MinHammingDifference": 1
    settings.MonoSettings.MonoMapInitializationSettings.NewMapPointsCreationSettings.AssociateMatcherSettings.MinHammingDifference = 1;
    //           }
    //         }
    //       }
    //     }
    //   }
    // }*/
}

void main()
{
    mage::MageSlamSettings settings{};
    SetDefaultSettings(settings);

    mage::Size imageSize{ 320, 180 };
    mage::Matrix extrinsics{};
    mage::MAGESlam::CameraConfiguration configuration{ mage::CameraIdentity::MONO, imageSize, mage::PixelFormat::GRAYSCALE8, extrinsics };
    auto configurations = gsl::make_span<mage::MAGESlam::CameraConfiguration>(&configuration, 1);

    mage::device::IMUCharacterization imuCharacterization{};

    auto slam = std::make_unique<mage::MAGESlam>(settings, configurations, imuCharacterization);

    std::vector<float> distortionCoefficients(5, 0);
    distortionCoefficients[0] = 0.094f;
    distortionCoefficients[1] = -0.35f;
    distortionCoefficients[2] = 0.42f;
    mage::calibration::LinearFocalLengthModel model{ 
        mage::calibration::Line{ -0.000111f, 0.81878f },
        mage::calibration::Line{ -0.000188f, 1.4517f },
        0.5064f,
        0.5115f,
        mage::calibration::Bounds{ 550, 700 }, 
        mage::Size{ 1920, 1080 },
        distortionCoefficients };
    std::shared_ptr<const mage::calibration::Poly3KCameraModel> cameraModel =
        model.CreatePoly3kCameraModel(650, 320, 180);

    mira::CameraSettings cameraSettings{};

    uint64_t idx = 0;
    std::vector<mage::FrameId> frameIds{};

    ProcessFrames(INPUT_MP4, [&](cv::Mat mat)
    {
        std::chrono::system_clock::time_point timePoint{ std::chrono::milliseconds(33 * idx) };
        frameIds.emplace_back(idx++, mage::CameraIdentity::MONO);
        mage::MAGESlam::FrameFormat format{ frameIds.back(), cameraModel, timePoint, cameraSettings };

        auto pixels = gsl::make_span<uint8_t>(mat.data, mat.rows * mat.cols);
        
        mage::MAGESlam::Frame frame{ format, pixels };
        auto future = slam->ProcessFrame(frame);
        auto result = future.get();
        std::cout << "Is pose good? " << (result.IsPoseGood() ? "Yes!" : "Not yet.") << std::endl;
    });
    
    auto fossil = mage::MAGESlam::Fossilize(std::move(slam));
    ExportFossilCsv(*fossil, frameIds);
}
