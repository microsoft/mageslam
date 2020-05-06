// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Fuser.h"
#include "Tracking/FeatureMatcher.h"
#include "Tracking/Reprojection.h"
#include "Utils/cv.h"
#include "Utils/Logging.h"
#include "Utils/MageConversions.h"
#include <boost/format.hpp>

namespace mage
{
    Fuser::Fuser(const FuserSettings& fuserSettings, const device::IMUCharacterization& imuCharacterization,
        const PoseEstimationSettings& poseSettings) :
        m_minDeltaPoseUpdateTime(std::chrono::duration_cast<SensorSample::Timestamp::duration>(std::chrono::milliseconds(fuserSettings.MinDeltaPoseRateMS))),
        m_maxDeltaPoseUpdateTime(std::chrono::duration_cast<SensorSample::Timestamp::duration>(std::chrono::milliseconds(fuserSettings.MaxDeltaPoseRateMS))),
        m_mode{ FuserMode::Invalid },
        m_modeBeforeLost{ FuserMode::Invalid },
        m_deltaStartPose(cv::Matx44f::eye()),
        m_deltaStartCovariance(cv::Matx66d::zeros()),
        m_deltaStartTimestamp{},
        m_fuserSettings(fuserSettings), 
        m_poseSettings(poseSettings),
        m_filterType{ FilterType(fuserSettings.FilterType.value) },
        m_worldIMUToWorldMageRotationOnly(cv::Matx33f::eye())
    {
        // TODO: Fuser throw std::runtime_error{ "Fuser stub." };
    }

    // called when the first mage pose is returned as we wont have a pair of poses available
    void Fuser::ResetDeltaPoseIntegration(const mage::SensorSample::Timestamp& curFrameTimestamp)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    SensorSample::Timestamp Fuser::GetResetDeltaPoseTimestamp() const
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void  Fuser::UpdateWithPose(const Pose& curPose, const SensorSample::Timestamp& curFrameTimestamp, const cv::Matx66d& poseCovariance)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void  Fuser::UpdateWithPose(const Pose& curPose, const SensorSample::Timestamp& curFrameTimestamp, double curPoseStdDev)
    {
        throw std::runtime_error{ "Fuser stub." };
    }
   
    bool Fuser::PublishFusedPoseEstimate(const SensorSample::Timestamp& curFrameTimestamp, Pose& fusedPose)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    //returns whether the sample queue accepted the image fence
    bool Fuser::AddImageFence(const mage::SensorSample::Timestamp& frameTimestamp)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void Fuser::RemoveImageFence(const mage::SensorSample::Timestamp& frameTimestamp)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void Fuser::SetMapOrigin(const Pose& magePose, const mage::SensorSample::Timestamp& mageTimestamp, const cv::Matx66d& poseCovariance)
    {
        throw std::runtime_error{ "Fuser stub." };
    }
    
    // our fuser runs estimating camera body, not imu body (since the camera pose is scaled in an unknown way, we cannot apply
    // a transformation in meters to describe the offset between the camera body and imu body
    bool Fuser::AddSample(const SensorSample& sample)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void Fuser::TransformDirIMUWorldToMAGEWorld(const cv::Vec3f& imuDir, cv::Vec3f& mageDir) const
    {
        throw std::runtime_error{ "Fuser stub." };
    }
      
    bool Fuser::GetMageWorldGravity(cv::Vec3f& gravInMageWorld)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    bool Fuser::HasGoodScale() const
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    bool Fuser::GetMageToMetersWorldScale(float& scaleMAGEToMeters)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void Fuser::ProcessSamplesToFence(const mage::SensorSample::Timestamp& fenceTimestamp)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    cv::Matx<float, 1, 6> Fuser::CalculateJacobian(const cv::Matx33f& calibrationMatrix, const cv::Matx31f& cameraSpacePt, const cv::Point2f& predictedImagePt, const cv::Point2f& observedImagePt)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    bool Fuser::EstimatePoseCovariance(const TrackingFrameHistory& referenceFrames, 
        const AnalyzedImage& currentFrame, 
        const Pose& estimatedPose,
        thread_memory memory,
        cv::Matx66d& covariance)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    FuserMode Fuser::GetMode() const
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void Fuser::SetMode(FuserMode newMode, const SensorSample::Timestamp& curFrameTimestamp)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    bool Fuser::HasGoodGravity() const
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    bool Fuser::CalculateResiduals(
        const TrackingFrameHistory& referenceFrames,
        const AnalyzedImage& currentFrame,
        const Pose& estimatedPose,
        thread_memory memory,
        std::vector<cv::Matx31f>& cameraSpacePts,
        std::vector<cv::Point2f>& predictedPts,
        std::vector<cv::Point2f>& observedPts,
        std::vector<float>& residuals)
    {
        throw std::runtime_error{ "Fuser stub." };
    }

    void Fuser::SwitchFilterOriginToMetricMage()
    {
        throw std::runtime_error{ "Fuser stub." };
    }    
}
