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
        switch (m_filterType)
        {
        case FilterType::NONE:
            break;
        case FilterType::FUSER3DOF:
        case FilterType::FUSER6DOF:
            m_filter3Dof = std::make_unique<SensorFilter3Dof>(imuCharacterization);
            m_activeFilter = m_filter3Dof.get();
            m_filter6Dof = nullptr;
            m_filterSimple6Dof = nullptr;
            break;
        case FilterType::SIMPLE6DOF:
            m_filterSimple6Dof = std::make_unique<SensorFilterSimple6Dof>(imuCharacterization);
            m_activeFilter = m_filterSimple6Dof.get();
            m_filter3Dof = nullptr;
            m_filter6Dof = nullptr;
            break;
        default:
            assert(false && "unexpected filter type");
            break;
        }
        assert(!imuCharacterization.useMagnetometer && "Transformation of magnetometer samples not implemented yet"); // it is not collocated with accel/gyro on this platform

        m_bodyIMUToBodyCamera = cv::Matx44f(imuCharacterization.BodyIMUToBodyCamera.data());
        m_bodyIMUToBodyCameraRotationOnly = m_bodyIMUToBodyCamera.get_minor<3, 3>(0, 0);
        m_bodyCameraToBodyIMU = cv::Matx44f(imuCharacterization.BodyCameraToBodyIMU.data());
    }

    // called when the first mage pose is returned as we wont have a pair of poses available
    void Fuser::ResetDeltaPoseIntegration(const mage::SensorSample::Timestamp& curFrameTimestamp)
    {
        SCOPE_TIMER(Fuser::ResetDeltaPoseIntegration);
        assert(this->m_filter6Dof->GetResetDeltaPoseTimestamp() <= curFrameTimestamp);
        m_filter6Dof->ResetDeltaPoseIntegration(curFrameTimestamp);

        assert(curFrameTimestamp >= m_deltaStartTimestamp && "not expecting a delta start prior to current");
        m_deltaStartTimestamp = curFrameTimestamp; //this is a hard reset, need to skip the first sample again
        m_deltaStartPose = cv::Matx44f::eye();
        m_deltaStartCovariance = cv::Matx66d::zeros();
    }

    SensorSample::Timestamp Fuser::GetResetDeltaPoseTimestamp() const
    {
        return m_filter6Dof->GetResetDeltaPoseTimestamp();
    }

    void  Fuser::UpdateWithPose(const Pose& curPose, const SensorSample::Timestamp& curFrameTimestamp, const cv::Matx66d& poseCovariance)
    {
        SCOPE_TIMER(Fuser::UpdateWithPose);

        assert(curFrameTimestamp >= m_deltaStartTimestamp && "Ensure timestamp is not from the past");
        assert(m_deltaStartTimestamp != SensorSample::Timestamp{} && "not reset before update");
        
        if (m_filter6Dof)
        {
            if (curFrameTimestamp == m_deltaStartTimestamp)
            {
                //a reset was called prior to this update, this completes the information needed to start the delta pose
                m_deltaStartPose = curPose;
                m_deltaStartCovariance = poseCovariance;
            }
            else if (std::chrono::duration_cast<std::chrono::microseconds>(curFrameTimestamp - m_deltaStartTimestamp) >= std::chrono::duration_cast<std::chrono::microseconds>(m_maxDeltaPoseUpdateTime))
            {
                //too much time has passed between frames for this to be a valid delta (dropped frames), reset with this as the new start
                ResetDeltaPoseIntegration(curFrameTimestamp);
                m_deltaStartPose = curPose;
                m_deltaStartCovariance = poseCovariance;
            }
            else if (std::chrono::duration_cast<std::chrono::microseconds>(curFrameTimestamp - m_deltaStartTimestamp) >= std::chrono::duration_cast<std::chrono::microseconds>(m_minDeltaPoseUpdateTime))
            {
                //completing the delta pose, send info to filter and then do an implicit reset (filter resets itself on an addvisualposedeltaupdate)
                assert(m_deltaStartTimestamp == GetResetDeltaPoseTimestamp() && "Delta being passed doesn't match filter's delta");
                cv::Matx44f cam0Tocam1 = To4x4(curPose.GetViewMatrix()) * m_deltaStartPose.GetInverseViewMatrix();
                m_filter6Dof->AddVisualPoseDeltaUpdate(ArrayFromMat(cam0Tocam1), curFrameTimestamp, ArrayFromMat(poseCovariance));
                
                m_deltaStartPose = curPose;
                m_deltaStartCovariance = poseCovariance;
                m_deltaStartTimestamp = curFrameTimestamp;
            }
            else
            {
                //don't add update, not enough time has passed
                return;
            }
        }
        else if(m_filter3Dof)
        {
            cv::Matx44f matWorldMageToCamera = To4x4(curPose.GetViewMatrix());
            m_filter3Dof->AddVisualRotationUpdate(ArrayFromMat(matWorldMageToCamera.get_minor<3,3>(0,0)), curFrameTimestamp, ArrayFromMat(poseCovariance.get_minor<3, 3>(3, 3)));
        }

        //TODO m_simple6dof may need to implement AddVisual methods in the future
    }

    void  Fuser::UpdateWithPose(const Pose& curPose, const SensorSample::Timestamp& curFrameTimestamp, double curPoseStdDev)
    {
        SCOPE_TIMER(Fuser::UpdateWithPose);
        UpdateWithPose(curPose, curFrameTimestamp, cv::Matx66f::eye() * (curPoseStdDev*curPoseStdDev));
    }
   
    bool Fuser::PublishFusedPoseEstimate(const SensorSample::Timestamp& curFrameTimestamp, Pose& fusedPose)
    {
        //TODO Need to revisit here to handle this timestamp
        UNUSED(curFrameTimestamp);

        SCOPE_TIMER(Fuser::PublishFusedPoseEstimate);

        if (m_activeFilter == m_filter6Dof.get())
        {
            if (!m_filter6Dof->IsValid() || m_deltaStartTimestamp == SensorSample::Timestamp{})
                return false;
        }

        assert(m_activeFilter->GetLastProcessedTimestamp() == curFrameTimestamp && "fuser not at expected time");

        std::array<double, 6 * 6> covariance;
        const cv::Matx44f fuserPose(m_activeFilter->GetFilteredCameraToWorldIMU(covariance).data());
        fusedPose = Pose(fuserPose);

        return true;
    }

    //returns whether the sample queue accepted the image fence
    bool Fuser::AddImageFence(const mage::SensorSample::Timestamp& frameTimestamp)
    {
        SCOPE_TIMER(Fuser::AddImageFence);

        //if in a mode that will use image fences
        bool added = false;
        added = m_sampleQueue.AddFence(frameTimestamp);
        
        return added;
    }

    void Fuser::RemoveImageFence(const mage::SensorSample::Timestamp& frameTimestamp)
    {
        SCOPE_TIMER(Fuser::RemoveImageFence);

        m_sampleQueue.RemoveFence(frameTimestamp);
    }

    void Fuser::SetMapOrigin(const Pose& magePose, const mage::SensorSample::Timestamp& mageTimestamp, const cv::Matx66d& poseCovariance)
    {
        SCOPE_TIMER(Fuser::SetMapOrigin);

        if (m_activeFilter != m_filterSimple6Dof.get())
        {
            // collect mage information
            m_originTimestamp = mageTimestamp;
            m_originWorldMageToCamera = To4x4(magePose.GetViewMatrix());
            m_originWorldMageToCameraCovariance = poseCovariance;

            //collect imu information
            std::array<double, 6 * 6> cameraToWorldImuPoseCovariance;
            {
                std::shared_lock<std::shared_mutex> guard(m_filterMutex);
                assert(m_activeFilter->GetLastProcessedTimestamp() == mageTimestamp && "fuser not at expected time");
                m_originCameraToWorldIMU = cv::Matx44f(m_activeFilter->GetFilteredCameraToWorldIMU(cameraToWorldImuPoseCovariance).data());
            }
            m_originCameraToWorldIMUCovariance = cv::Matx66d(cameraToWorldImuPoseCovariance.data());

            //calculate conversions
            const cv::Matx44f worldMageToWorldIMU = m_originCameraToWorldIMU * m_originWorldMageToCamera;

            //uses the transpose of the inverse (safe practice for transforming directions, the intended use of this matrix)
            m_worldIMUToWorldMageRotationOnly = worldMageToWorldIMU.get_minor<3, 3>(0, 0).t();
        }
        else
        {
            cv::Matx44f firstTrackingWorldToMageWorldRotationOnly = magePose.GetInverseViewMatrix();
            firstTrackingWorldToMageWorldRotationOnly(0, 3) = 0;
            firstTrackingWorldToMageWorldRotationOnly(1, 3) = 0;
            firstTrackingWorldToMageWorldRotationOnly(2, 3) = 0;

            m_filterSimple6Dof->SimpleSwitchFilterOrigin(ArrayFromMat(firstTrackingWorldToMageWorldRotationOnly), mageTimestamp);
        }

        isMapOriginSet = true;
    }
    
    // our fuser runs estimating camera body, not imu body (since the camera pose is scaled in an unknown way, we cannot apply
    // a transformation in meters to describe the offset between the camera body and imu body
    bool Fuser::AddSample(const SensorSample& sample)
    {
        {
            std::shared_lock<std::shared_mutex> guard(m_filterMutex);
            if (!m_activeFilter->IsValid())
                return false;
        }

        SCOPE_TIMER(Fuser::AddSample);

        //modify forces applied as if measured at camera 
        switch (sample.GetType())
        {       
        case SensorSample::SampleType::Accelerometer:
        {
            //CameraInIMUFrame = CameraToIMU * cv::Vec4f(0, 0, 0, 1);
            const cv::Vec3f CameraInIMUFrame(m_bodyCameraToBodyIMU(0, 3), m_bodyCameraToBodyIMU(1, 3), m_bodyCameraToBodyIMU(2, 3));

            // can't use angular acceleration term because it is too noisy, the forces are only partially transformed
            // to the location of the camera sensor
            cv::Vec3f angVel;
            {
                SensorSample::Timestamp timeAngVel;
                std::shared_lock<std::shared_mutex> guard(m_filterMutex);
                angVel = cv::Vec3f(m_activeFilter->GetBodyAngularVelocity(timeAngVel).data());
            }

            cv::Vec3f linAccelDueToCentripetal = angVel.cross(angVel.cross(CameraInIMUFrame));
            cv::Vec3f cameraBodySample = m_bodyIMUToBodyCameraRotationOnly * (cv::Vec3f(sample.GetData().data()) + linAccelDueToCentripetal);
            return m_sampleQueue.AddSample({ SensorSample::SampleType::Accelerometer, sample.GetTimestamp(),{ cameraBodySample[0],cameraBodySample[1],cameraBodySample[2] } });
        }
        case SensorSample::SampleType::Gyrometer:
        {
            const cv::Vec3f cameraBodySample = m_bodyIMUToBodyCameraRotationOnly * cv::Vec3f(sample.GetData().data());
            return m_sampleQueue.AddSample({ SensorSample::SampleType::Gyrometer, sample.GetTimestamp(),{ cameraBodySample[0],cameraBodySample[1],cameraBodySample[2] } });
        }
        case SensorSample::SampleType::Magnetometer:
            assert(false && "Transformation of magnetometer samples not implemented yet");
            return false;
        default:
            assert(false && "Unexpected sample type in add sample");
            return false;
        }
    }

    void Fuser::TransformDirIMUWorldToMAGEWorld(const cv::Vec3f& imuDir, cv::Vec3f& mageDir) const
    {
        SCOPE_TIMER(Fuser::TransformDirIMUWorldToMAGEWorld);

        assert(MapOriginValid() && "Can't calculate direction of mage origin not set");

        // really asking for mageWorldToCamera = worldIMUToWorldMAGE * imuDir
        mageDir = m_worldIMUToWorldMageRotationOnly * cv::normalize(imuDir);
    }
      
    bool Fuser::GetMageWorldGravity(cv::Vec3f& gravInMageWorld)
    {
        SCOPE_TIMER(Fuser::GetMageWorldGravity);

        std::array<float, 3> gravInIMUWorldV;
        
        bool success = false;

        {
            std::shared_lock<std::shared_mutex> guard(m_filterMutex);

            success = m_activeFilter->GetWorldGravity(gravInIMUWorldV);
        }

        cv::Vec3f gravInIMUWorld(gravInIMUWorldV.data());
        TransformDirIMUWorldToMAGEWorld(gravInIMUWorld, gravInMageWorld);

        return success;
    }

    bool Fuser::HasGoodScale() const
    {
        if (m_filter6Dof == nullptr)
        {
           return false;
        }

        return m_filter6Dof->HasGoodScale();
    }

    bool Fuser::GetMageToMetersWorldScale(float& scaleMAGEToMeters)
    {
        if (m_filter6Dof == nullptr)
        {
            scaleMAGEToMeters = 0.0f;
            return false;
        }
        
        return m_filter6Dof->GetEstimatedVisualToMetersScale(scaleMAGEToMeters);
    }

    void Fuser::ProcessSamplesToFence(const mage::SensorSample::Timestamp& fenceTimestamp)
    {
        SCOPE_TIMER(Fuser::ProcessSamplesToFence);

        // pull off each set of samples at the same timestamp up to the first image fence
        std::vector<SensorSample> correlatedSamples;
        correlatedSamples.reserve(3);
        SensorSample::Timestamp lastProcessedTimestamp;
        
        while (m_sampleQueue.PopCorrelatedSamples(correlatedSamples))
        {
            assert(correlatedSamples.empty() || (correlatedSamples.front().GetTimestamp() <= fenceTimestamp && "Samples ahead of fence"));

            {
                std::unique_lock<std::shared_mutex> guard(m_filterMutex);  
                lastProcessedTimestamp = m_activeFilter->ProcessCorrelatedSamples(correlatedSamples); 
                assert(lastProcessedTimestamp <= fenceTimestamp && "Filter ran ahead");
            }
        }

        {
            std::unique_lock<std::shared_mutex> guard(m_filterMutex);
            m_activeFilter->PredictTo(fenceTimestamp);
        }
    }

    cv::Matx<float, 1, 6> Fuser::CalculateJacobian(const cv::Matx33f& calibrationMatrix, const cv::Matx31f& cameraSpacePt, const cv::Point2f& predictedImagePt, const cv::Point2f& observedImagePt)
    {
        float fx = calibrationMatrix(0,0); //focal length x;
        float fy = calibrationMatrix(1,1); //focal length y

        float pcx = cameraSpacePt(0, 0);
        float pcy = cameraSpacePt(1, 0);
        float pcz = cameraSpacePt(2, 0);

        // this is the jacobian of the dimagePt/dcamerapt * jacobian of the dcameraPt/dPose
        cv::Matx<float, 2, 6> j;
        j(0, 0) = fx / pcz;       j(0, 1) = 0;          j(0, 2) = -pcx * fx / (pcz*pcz);    j(0, 3) = pcy * -pcx * fx / (pcz*pcz);          j(0, 4) = fx + pcx*pcx*fx / (pcz*pcz);      j(0, 5) = -pcy * fx / pcz;
        j(1, 0) = 0;              j(1, 1) = fy / pcz;   j(1, 2) = -pcy * fy / (pcz*pcz);    j(1, 3) = -fy + pcy * pcy * fy / (pcz*pcz);     j(1, 4) = -pcx * -pcy * fy / (pcz*pcz);     j(1, 5) = pcx * fy / pcz;

        // this is the jacobian of the dresidual/dimagePt (the L2Norm portion)
        //https://math.stackexchange.com/questions/883016/gradient-of-l2-norm-squared
        cv::Matx<float, 1, 2> jNorm;
        cv::Point2f dImgPt = predictedImagePt - observedImagePt;
        jNorm(0, 0) = 2.0f * dImgPt.x;  
        jNorm(0, 1) = 2.0f * dImgPt.y;

        return jNorm * j;
    }

    bool Fuser::EstimatePoseCovariance(const TrackingFrameHistory& referenceFrames, 
        const AnalyzedImage& currentFrame, 
        const Pose& estimatedPose,
        thread_memory memory,
        cv::Matx66d& covariance)
    {
        SCOPE_TIMER(Fuser::UpdateWithPose);

        //calculate residuals
        std::vector<float> residuals;
        std::vector<cv::Matx31f> cameraSpacePts;
        std::vector<cv::Point2f> predictedPts;
        std::vector<cv::Point2f> observedPts;

        if (!CalculateResiduals(referenceFrames, currentFrame, estimatedPose, memory, cameraSpacePts, predictedPts, observedPts, residuals))
            return false;
    
        //calculate jacobian of objective function with respect to pose
        cv::Mat jacobian(gsl::narrow_cast<int>(residuals.size()), 6, CV_32F);
        for (size_t mapPtIdx=0; mapPtIdx < residuals.size(); mapPtIdx++)
        {
            cv::Mat dst = jacobian.row(gsl::narrow_cast<int>(mapPtIdx));
            cv::Mat src(CalculateJacobian(currentFrame.GetUndistortedCalibration().GetCameraMatrix(), cameraSpacePts[mapPtIdx], predictedPts[mapPtIdx], observedPts[mapPtIdx]));
            src.copyTo(dst);
        }

        //approximate Hessian (gauss-newton)
        cv::Mat hessian(6, 6, CV_32F);
        hessian = jacobian.t() * jacobian;
        
        cv::Matx66d hessianMatx = (cv::Matx<double, 6, 6>)hessian;

        //estimate covariance
        bool inverted = false;
        cv::Matx66d cov = hessianMatx.inv(0,&inverted);
        if (!inverted)
        {
            assert(false && "failed to invert mtatrix");
            return false;
        }

        covariance = cv::Matx66d(cov);
      
        return true;
    }

    FuserMode Fuser::GetMode() const
    {
        std::shared_lock<std::shared_mutex> guard(m_modeMutex);
        return m_mode;
    }

    void Fuser::SetMode(FuserMode newMode, const SensorSample::Timestamp& curFrameTimestamp)
    {       
        std::shared_lock<std::shared_mutex> guard(m_modeMutex);
        SCOPE_TIMER(Fuser::SetMode);

        switch (newMode)
        {     
        case mage::FuserMode::Invalid:
            LogMessage<>((boost::wformat(L"Fuser Invalid %d") % curFrameTimestamp.time_since_epoch().count()).str());
            m_activeFilter = nullptr;
            m_filter3Dof.release();
            m_filter6Dof.release();
            m_filterSimple6Dof.release();
            break;
        case mage::FuserMode::WaitForMageInit:
            LogMessage<>((boost::wformat(L"Fuser WaitForMageInit %d") % curFrameTimestamp.time_since_epoch().count()).str());
            assert(m_filter6Dof.get() == nullptr);
            {
                std::unique_lock<std::shared_mutex> filterGuard(m_filterMutex);
                m_activeFilter->Reset(curFrameTimestamp);
            }
            break;
        case mage::FuserMode::WaitForGravityConverge:
            LogMessage<>((boost::wformat(L"Fuser WaitForGravityConverge %d") % curFrameTimestamp.time_since_epoch().count()).str());
            isMapOriginSet = false;
            break;
        case mage::FuserMode::VisualTrackingLost:
            if(m_mode != FuserMode::VisualTrackingLost)
            {
                LogMessage<>((boost::wformat(L"Fuser VisualTrackingLost %d") % curFrameTimestamp.time_since_epoch().count()).str());
                m_modeBeforeLost = m_mode;
            }
            break;
        case mage::FuserMode::VisualTrackingReacquired:
            assert(m_mode == FuserMode::VisualTrackingLost && "Expected to be lost before reloc");
            LogMessage<>((boost::wformat(L"Fuser VisualTrackingReacquired %d") % curFrameTimestamp.time_since_epoch().count()).str());
            newMode = m_modeBeforeLost;
            if (m_activeFilter == m_filter6Dof.get())
            {
                ResetDeltaPoseIntegration(curFrameTimestamp);
            }

            //TODO m_simple6dof havenot decided how to handle the tracking lost scenario

            m_modeBeforeLost = FuserMode::Invalid;
            break;
        case mage::FuserMode::ScaleInit:
            LogMessage<>((boost::wformat(L"Fuser ScaleInit %d") % curFrameTimestamp.time_since_epoch().count()).str());
            {
                std::unique_lock<std::shared_mutex> filterGuard(m_filterMutex);
                m_filter6Dof = std::make_unique<SensorFilter6Dof>(std::move(m_filter3Dof));
            }
            m_deltaStartTimestamp = curFrameTimestamp; //implicit reset by fuser setup
            m_activeFilter = m_filter6Dof.get();
            break;
        case mage::FuserMode::Tracking:            
            LogMessage<>((boost::wformat(L"Fuser Tracking %d") % curFrameTimestamp.time_since_epoch().count()).str());
            SwitchFilterOriginToMetricMage();
            break;
        default:
            assert(false && "unexpected state");
            break;
        }

        m_mode = newMode;
    }

    bool Fuser::HasGoodGravity() const
    {
        std::shared_lock<std::shared_mutex> guard(m_filterMutex);
        return m_activeFilter->HasGoodGravity();
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
        SCOPE_TIMER(PoseEstimator::CalculateResiduals);

        residuals.clear();

        size_t referencePointCount = std::accumulate(referenceFrames.begin(), referenceFrames.end(), (size_t)0,
            [](size_t sum, const auto& frame) { return sum + frame.Keyframe->GetMapPointCount(); });

        if (referencePointCount == 0)
        {
            return false;
        }

        auto referenceMapPoints = memory.stack_vector<MapPointTrackingProxy::ViewT>(referencePointCount);
        auto referenceKeypoints = memory.stack_vector<cv::KeyPoint>(referencePointCount);
        auto referenceDescriptors = memory.stack_vector<ORBDescriptor::Ref>(referencePointCount);

        auto referenceMapPointsSet = memory.stack_unique_vector<Id<MapPoint>>(referencePointCount);

        const cv::Matx34f& viewMatrix = estimatedPose.GetViewMatrix();
        const cv::Matx33f& cameraMatrix = currentFrame.GetUndistortedCalibration().GetCameraMatrix();

        std::vector<cv::Matx31f> predictedCameraSpacePositions;
        std::vector<cv::Point2f> predictedImageSpacePositions;

        for (const auto& referenceFrame : referenceFrames)
        {
            referenceFrame.Keyframe->IterateAssociations([&](const MapPointTrackingProxy& proxy, KeypointDescriptorIndex idx)
            {
                const auto& id = proxy.GetId();
                auto itr = referenceMapPointsSet.find(id);
                if (itr == referenceMapPointsSet.end())
                {
                    referenceMapPointsSet.insert(proxy.GetId());

                    const Projection projection = ProjectUndistorted(viewMatrix, cameraMatrix, proxy.GetPosition());
                    if (projection.Distance >= 0)
                    {
                        predictedCameraSpacePositions.emplace_back(viewMatrix * cv::Matx41f{ proxy.GetPosition().x, proxy.GetPosition().y, proxy.GetPosition().z, 1.0f });
                        predictedImageSpacePositions.emplace_back(projection.Point);
                        referenceMapPoints.emplace_back(proxy);
                        const auto& analyzedImage = referenceFrame.Keyframe->GetAnalyzedImage();
                        referenceKeypoints.emplace_back(analyzedImage->GetKeyPoint(idx));
                        referenceDescriptors.emplace_back(analyzedImage->GetDescriptor(idx));
                    }
                }
            });
        }

        residuals.clear();
        residuals.reserve(referenceMapPoints.size());
        predictedPts.clear();
        predictedPts.reserve(referenceMapPoints.size());
        observedPts.clear();
        observedPts.reserve(referenceMapPoints.size());

        std::vector<bool> queryMask(referenceKeypoints.size(), true);
        std::vector<bool> trainMask(currentFrame.GetKeyPoints().size(), true);
        
        float searchRadiusPixels = 1;
        while (searchRadiusPixels < currentFrame.GetWidth() && residuals.size() < referenceMapPoints.size())
        {
            std::vector<cv::DMatch> good_matches;

            // search for matches in predicted location of keypoints
            RadiusMatch(
                referenceKeypoints,                             //query keypoint
                &predictedImageSpacePositions,                  //query keypoint overrides
                &queryMask,                                     //query keypoint mask
                referenceDescriptors,                           //query descriptors
                currentFrame.GetKeyPoints(),                   //target (train) keypoints
                currentFrame.GetKeypointSpatialIndex(),        //target (train) keypoints index
                &trainMask,                                     //target (train) Keypoints mask
                currentFrame.GetDescriptors(),                 //target (train) descriptors
                searchRadiusPixels,
                m_fuserSettings.OrbMatcherSettings.MaxHammingDistance,
                m_fuserSettings.OrbMatcherSettings.MinHammingDifference,
                memory,
                good_matches);
         
            for (const auto& match : good_matches)
            {
                const cv::Point2f& predictedPosition = predictedImageSpacePositions[match.queryIdx];
                const cv::Point2f& observedPosition = currentFrame.GetKeyPoint(match.trainIdx).pt;
                cv::Point2f deltaPosition = observedPosition - predictedPosition;
                residuals.push_back(sqrtf(deltaPosition.dot(deltaPosition)));
                cameraSpacePts.push_back(predictedCameraSpacePositions[match.queryIdx]);
                predictedPts.push_back(predictedPosition);
                observedPts.push_back(observedPosition);
                queryMask[match.queryIdx] = false;
                trainMask[match.trainIdx] = false;
            }

            searchRadiusPixels *= 2;
        }

        return residuals.size() > 0;
    }

    void Fuser::SwitchFilterOriginToMetricMage()
    {
        std::unique_lock<std::shared_mutex> guard(m_filterMutex);

        assert(isMapOriginSet && "can't change origin if no link between mage and imu");
        assert(m_activeFilter == m_filter6Dof.get() && "need 6dof with converged scale to call this");
        assert(m_filter6Dof->HasGoodScale() && "expecting converged scale");

        // clear out the bad position we have due to scale initialization
        m_filter6Dof->ResetPoseTranslation();

        //build pose and covariance for new origin        
        float scaleMageToMeters;
        m_filter6Dof->GetEstimatedVisualToMetersScale(scaleMageToMeters);
        float scaleMageToMetersSq = scaleMageToMeters * scaleMageToMeters;

        //scale the translation in the matrix linking the two, makes it mage world (metric) to camera
        cv::Matx44f metricWorldMageToCamera = m_originWorldMageToCamera;
        metricWorldMageToCamera(0, 3) *= scaleMageToMeters;
        metricWorldMageToCamera(1, 3) *= scaleMageToMeters;
        metricWorldMageToCamera(2, 3) *= scaleMageToMeters;

        cv::Matx66d metricWorldMageToCameraCovariance = m_originWorldMageToCameraCovariance;
        //don't scale rotation
        for (int idxRow = 0; idxRow <3; idxRow++)
        {
            for (int idxCol = 0; idxCol < 3; idxCol++)
            {
                metricWorldMageToCameraCovariance(idxRow, idxCol) *= scaleMageToMetersSq;
            }
        }
                
        //prepare imu portion of final matrix
       
        //build final matrix and covariance
        cv::Matx44f metricWorldMageToWorldIMU = m_originCameraToWorldIMU * metricWorldMageToCamera;
        cv::Matx66d metricWorldMageToWorldIMUCovariance(SensorFilter6Dof::GetCovarianceForMatMul(ArrayFromMat(m_originCameraToWorldIMU), ArrayFromMat(m_originCameraToWorldIMUCovariance), ArrayFromMat(metricWorldMageToCameraCovariance)).data());

        // change the fuser origin to match mage (this will transform map based state like pose and gravity to mage's coordinate frame but in meters)
        // the map change will be used on the right (eg. poseinmageOrigin = poseInimuorigin * mageOriginToImuOrigin)
        // should be mageOriginToImuOrigin. ImuOrigin should have zero translation, mage translation should be scaled to metric
        m_filter6Dof->SwitchFilterOrigin(ArrayFromMat(metricWorldMageToWorldIMU), ArrayFromMat(metricWorldMageToWorldIMUCovariance));

        // set the translation of the estimated pose
        cv::Matx31f metricMageWorldPos = m_deltaStartPose.GetWorldSpacePosition() * scaleMageToMeters;
        m_filter6Dof->SetPoseWorldPosition(metricMageWorldPos(0), metricMageWorldPos(1), metricMageWorldPos(2), ArrayFromMat(m_deltaStartCovariance.get_minor<3,3>(0,0)));

        //clear out the transformation matrices now
        m_originCameraToWorldIMU = cv::Matx44f::eye();
        m_originCameraToWorldIMUCovariance = cv::Matx66d::zeros();
        m_originWorldMageToCamera = cv::Matx44f::eye();
        m_originWorldMageToCameraCovariance = cv::Matx66d::zeros();
        m_worldIMUToWorldMageRotationOnly = cv::Matx33f::eye();
        
    }    
}
