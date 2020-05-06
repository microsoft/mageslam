// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "FuserWorker.h"

#include "Map/ThreadSafeMap.h"

#include "Fuser/Fuser.h"

namespace mage
{
    struct FuserWorker::Impl
    {
        mira::ticket_scope Registrations{};
        AnalyzedImage::time_stamp LastTimeStamp;
        bool Tracking = false;

        bool HasCovariance = false;
        cv::Matx66d Covariance;
        
        TrackingFrameHistory History;
        memory_pool MemoryPool{};

        bool WaitedForInit = false;
    };

    FuserWorker::FuserWorker(
        TrackingMediator& mediator,
        mira::determinator& ,
        Fuser& fuser,
        MageContext& /*context*/,
        const MageSlamSettings& settings)
        : m_impl{ std::make_unique<Impl>() }
    {
        if (!settings.FuserSettings.UseFuser)
            return;

        m_impl->Registrations += mediator.add_listener<AnalysisCompleted>([&](const AnalysisCompleted& frame)
        {
            if (!frame.Analyzed)
                return;

            m_impl->LastTimeStamp = frame.Analyzed->GetTimeStamp();

            if (!m_impl->WaitedForInit)
            {
                fuser.SetMode(FuserMode::WaitForMageInit, m_impl->LastTimeStamp);
                m_impl->WaitedForInit = true;
            }

            //catch up if needed
            fuser.ProcessSamplesToFence(m_impl->LastTimeStamp);

            //tick state machine
            if (m_impl->Tracking)
            {
                if (settings.FuserSettings.FilterType == FilterType::FUSER6DOF && fuser.GetMode() == FuserMode::WaitForGravityConverge && fuser.HasGoodGravity())
                {
                    fuser.SetMode(FuserMode::ScaleInit, frame.Analyzed->GetTimeStamp());
                }
                else if (fuser.GetMode() == FuserMode::ScaleInit && fuser.HasGoodScale())
                {
                    fuser.SetMode(FuserMode::Tracking, frame.Analyzed->GetTimeStamp());
                }
                else if (fuser.GetMode() == FuserMode::VisualTrackingLost)
                {
                    fuser.SetMode(FuserMode::VisualTrackingReacquired, frame.Analyzed->GetTimeStamp());
                }
            }
        });

        m_impl->Registrations += mediator.add_listener<InitCompleted>([&](const InitCompleted&)
        {
            // mage initialized kick off fuser gravity converge
            fuser.SetMode(FuserMode::WaitForGravityConverge, m_impl->LastTimeStamp);

            m_impl->Tracking = true;
        });

        m_impl->Registrations += mediator.add_listener<std::shared_ptr<const PoseEstimated>>([&](const std::shared_ptr<const PoseEstimated>& estimate)
        {
            FuserMode fuserMode = fuser.GetMode();
            bool calculateCovariance = settings.FuserSettings.ApplyVisualUpdate &&
                (fuserMode == FuserMode::WaitForGravityConverge || fuserMode == FuserMode::ScaleInit || fuserMode == FuserMode::Tracking);

            if (calculateCovariance)
            {
                auto memory = m_impl->MemoryPool.create();

                m_impl->HasCovariance = fuser.EstimatePoseCovariance(
                    m_impl->History, *estimate->Frame->GetAnalyzedImage(),
                    estimate->Frame->GetPose(), memory, m_impl->Covariance);

                // TODO wouldn't it be better to set the map origin to the refined pose rather than the estimated one?
                // TLM changes the pose quite a bit.
                if (m_impl->HasCovariance && !fuser.MapOriginValid())
                {
                    fuser.SetMapOrigin(estimate->Frame->GetPose(), estimate->Frame->GetAnalyzedImage()->GetTimeStamp(), m_impl->Covariance);
                }
            }
        });

        m_impl->Registrations += mediator.add_listener<HistoryUpdated>([&](const HistoryUpdated& update)
        {
            m_impl->History = update.History;
        });

        m_impl->Registrations += mediator.add_listener<TrackingLost>([&](const TrackingLost&)
        {
            if (m_impl->Tracking)
            {
                fuser.SetMode(FuserMode::VisualTrackingLost, m_impl->LastTimeStamp);
                m_impl->Tracking = false;
            }

            m_impl->History.clear();
        });

        m_impl->Registrations += mediator.add_listener<PoseRefined>([&](const PoseRefined& result)
        {
            // update the fuser with mage's latest visual pose (if it has initialized)
            auto fuserMode = fuser.GetMode();
            if (fuserMode == FuserMode::ScaleInit || fuserMode == FuserMode::Tracking)
            {
                if (!fuser.MapOriginValid())
                {
                    assert(m_impl->HasCovariance && "need covariance");
                    fuser.SetMapOrigin(result.Frame->GetPose(), result.Frame->GetAnalyzedImage()->GetTimeStamp(), m_impl->Covariance);
                }

                if (m_impl->HasCovariance)
                {
                    fuser.UpdateWithPose(result.Frame->GetPose(), result.Frame->GetAnalyzedImage()->GetTimeStamp(), m_impl->Covariance);
                }
                else
                {
                    fuser.UpdateWithPose(result.Frame->GetPose(), result.Frame->GetAnalyzedImage()->GetTimeStamp(), settings.FuserSettings.StdDevPoseError);
                }
            }
        });
    }

    FuserWorker::~FuserWorker() = default;
}
