// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "FuserLib.h"
#include <MageSlam.h>
#include <FuserLib.h>
#include <data\Pose.h>
#include <opencv2\core\core.hpp>
#include "Tracking\PoseEstimator.h"

namespace UnitTests
{
    class FuserUnitTest;
}

namespace mage
{
    class Fuser
    {
    public:
        Fuser(const FuserSettings& fuserSettings, 
            const device::IMUCharacterization& imuCharacterization, 
            const PoseEstimationSettings& poseSettings);

        void UpdateWithPose(const Pose& curPose, const SensorSample::Timestamp& curFrameTimestamp, const cv::Matx66d& poseCovariance);
        void UpdateWithPose(const Pose& curPose, const SensorSample::Timestamp& curFrameTimestamp, double curPoseStdDev);
       
        bool MapOriginValid() const { return isMapOriginSet; }

        void SetMapOrigin(const Pose& magePose, const mage::SensorSample::Timestamp& mageTimestamp, const cv::Matx66d& poseCovariance);
             
        bool AddImageFence(const mage::SensorSample::Timestamp& frameTimestamp);
        void RemoveImageFence(const mage::SensorSample::Timestamp& frameTimestamp);
        bool AddSample(const SensorSample& sample);

        bool PublishFusedPoseEstimate(const SensorSample::Timestamp& curFrameTimestamp, Pose& fusedMagePose);
        
        bool HasGoodGravity() const;
        bool GetMageWorldGravity(cv::Vec3f& gravInMageWorld);
        
        bool HasGoodScale() const;
        bool GetMageToMetersWorldScale(float& scaleMAGEToMeters);

        FuserMode GetMode() const;
        void SetMode(FuserMode newMode, const SensorSample::Timestamp& curFrameTimestamp);
        
        void ProcessSamplesToFence(const mage::SensorSample::Timestamp& fenceTimestamp);

        bool EstimatePoseCovariance(const mage::TrackingFrameHistory& referenceFrames,
            const AnalyzedImage& currentFrame,
            const mage::Pose& estimatedPose,
            mage::thread_memory memory,
            cv::Matx66d& covariance);

    private:
        SensorSample::Timestamp GetResetDeltaPoseTimestamp() const;
        void ResetDeltaPoseIntegration(const mage::SensorSample::Timestamp& curFrameTimestamp);
        
        void TransformDirIMUWorldToMAGEWorld(const cv::Vec3f& imuDir, cv::Vec3f& mageDir) const;

        void SwitchFilterOriginToMetricMage();

        cv::Matx<float, 1, 6> CalculateJacobian(const cv::Matx33f& calibrationMatrix, const cv::Matx31f& cameraSpacePt, const cv::Point2f& predictedImagePt, const cv::Point2f& observedImagePt);
        
        bool CalculateResiduals(
            const TrackingFrameHistory& referenceFrames,
            const AnalyzedImage& currentFrame,
            const Pose& estimatedPose,
            thread_memory memory,
            std::vector<cv::Matx31f>& cameraSpacePts,
            std::vector<cv::Point2f>& predictedPts,
            std::vector<cv::Point2f>& observedPts,
            std::vector<float>& residuals);

        SensorSampleQueue m_sampleQueue;

        ISensorFilter* m_activeFilter;
        std::unique_ptr<SensorFilter6Dof> m_filter6Dof;
        std::unique_ptr<SensorFilter3Dof> m_filter3Dof;
        std::unique_ptr<SensorFilterSimple6Dof> m_filterSimple6Dof;

        bool isMapOriginSet = false;
        FilterType m_filterType;

        SensorSample::Timestamp m_originTimestamp;
        cv::Matx44f m_originCameraToWorldIMU;
        cv::Matx66d m_originCameraToWorldIMUCovariance;
        cv::Matx44f m_originWorldMageToCamera;
        cv::Matx66d m_originWorldMageToCameraCovariance;     
        cv::Matx33f m_worldIMUToWorldMageRotationOnly;
      
        cv::Matx44f m_bodyIMUToBodyCamera;
        cv::Matx33f m_bodyIMUToBodyCameraRotationOnly;
        cv::Matx44f m_bodyCameraToBodyIMU;

        FuserMode m_mode;
        FuserMode m_modeBeforeLost;
        
        Pose m_deltaStartPose;
        SensorSample::Timestamp m_deltaStartTimestamp;
        cv::Matx66d m_deltaStartCovariance;
        
        mutable std::shared_mutex m_modeMutex;
        mutable std::shared_mutex m_filterMutex;

        SensorSample::Timestamp::duration m_minDeltaPoseUpdateTime;
        SensorSample::Timestamp::duration m_maxDeltaPoseUpdateTime;

        const PoseEstimationSettings& m_poseSettings;
        const FuserSettings& m_fuserSettings;

        friend class ::UnitTests::FuserUnitTest;
    };
}
