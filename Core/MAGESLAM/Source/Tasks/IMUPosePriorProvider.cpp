// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "IMUPosePriorProvider.h"

#include <arcana/threading/dispatcher.h>

namespace mage
{
    struct IMUPosePriorProvider::Impl
    {
        ImuMixin Imu;
        mira::background_dispatcher<32> Dispatcher;
    };

    IMUPosePriorProvider::IMUPosePriorProvider(ImuMixin imu)
        : BaseWorker{0, 0}
        , m_impl{ new Impl{
            imu
        } }
    {}

    IMUPosePriorProvider::~IMUPosePriorProvider() = default;

    void IMUPosePriorProvider::AddSample(const mage::SensorSample& sample)
    {
        Pending() += mira::make_task(m_impl->Dispatcher, Cancellation(), [this, sample]
        {
            m_impl->Imu.get<IIMUReceiver>()->AddSample(sample);
        });
    }

    void IMUPosePriorProvider::OnTrackingLost()
    {
        Pending() += mira::make_task(m_impl->Dispatcher, Cancellation(), [this]
        {
            m_impl->Imu.get<IVisualPoseReceiver>()->OnTrackingLost();
        });
    }

    mira::task<Pose> IMUPosePriorProvider::GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& timestamp)
    {
        return Pending() += mira::make_task(m_impl->Dispatcher, Cancellation(), [this, frame = history.newest(), timestamp]
        {
            m_impl->Imu.get<IPredictor>()->PredictUpTo(frame.Keyframe->GetAnalyzedImage()->GetTimeStamp());

            m_impl->Imu.get<IVisualPoseReceiver>()->AddPose(frame.UpdatedPose, frame.Keyframe->GetAnalyzedImage()->GetTimeStamp());

            m_impl->Imu.get<IPredictor>()->PredictUpTo(timestamp);

            return m_impl->Imu.get<IPoseEstimator>()->GetPose();
        });
    }
}
