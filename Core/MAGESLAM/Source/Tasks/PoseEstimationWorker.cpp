// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "PoseEstimationWorker.h"

#include "Utils\Logging.h"

#include "MageContext.h"

namespace mage
{
    struct PoseEstimationWorker::Impl
    {
        TrackingMediator& Mediator;
        mira::determinator& Determinator;
        mage::Fuser& Fuser;
        MageContext& Context;
        const MageSlamSettings& Settings;

        Impl(TrackingMediator& mediator, mira::determinator& determinator, mage::Fuser& fuser, MageContext& context, const MageSlamSettings& settings)
            : Mediator{ mediator }
            , Determinator{ determinator }
            , Fuser{ fuser }
            , Context{ context }
            , Settings{ settings }
        {}
    };

    PoseEstimationWorker::PoseEstimationWorker(
        TrackingMediator& mediator,
        mira::determinator& determinator,
        Fuser& fuser,
        MageContext& context,
        const MageSlamSettings& settings)
        :   BaseWorker{ 100 * 1024, 100 * 1024 },
            m_impl{ std::make_unique<Impl>(mediator, determinator, fuser, context, settings) }
    {}

    mira::task<std::shared_ptr<const PoseEstimated>> PoseEstimationWorker::EstimatePose(const FrameAnalyzed& frame, const TrackingFrameHistory& history, const boost::optional<Pose>& posePrior)
    {
        return mira::make_task(m_impl->Mediator.dispatcher(), Cancellation(), [this, frame, &history, posePrior]() -> mira::expected<std::shared_ptr<const PoseEstimated>>
        {
            // if we got an image with no keypoints or descriptors, we are lost
            if (posePrior && frame.Analyzed->GetDescriptorsCount() == 0)
            {
                DETERMINISTIC_CHECK(m_impl->Determinator, 3241);
                return mira::errc::failed;
            }

            std::shared_ptr<KeyframeBuilder> currentKeyframe = std::make_shared<KeyframeBuilder>(frame.Analyzed, Pose{});

            thread_memory memory = MemoryPool().create();

            if (posePrior)
            {
                PoseEstimator estimator{ m_impl->Settings.PoseEstimationSettings, m_impl->Settings.RelocalizationSettings };

                bool poseEstimationSucceeded = estimator.TryEstimatePoseWithPrior(
                    history, *posePrior, m_impl->Settings.PoseEstimationSettings.FeatureMatchThreshold, memory, *currentKeyframe);

                if (poseEstimationSucceeded)
                {
                    const auto& relativeKeyframe = *history.newest().Keyframe;
                    return std::make_shared<const PoseEstimated>(
                        std::move(currentKeyframe),
                        false,
                        relativeKeyframe.GetAnalyzedImage()->GetFrameId(),
                        relativeKeyframe.GetPose()
                    );
                }
            }
            else
            {
                std::vector<KeyframeProxy> candidates;
                m_impl->Context.Map.FindSimilarKeyframes(frame.Analyzed, candidates);

                PoseEstimator estimator{ m_impl->Settings.PoseEstimationSettings, m_impl->Settings.RelocalizationSettings };

                auto successIndex = estimator.TryEstimatePoseFromCandidates(m_impl->Context.BagOfWords, candidates, m_impl->Determinator, memory, *currentKeyframe);

                DETERMINISTIC_CHECK(m_impl->Determinator, currentKeyframe);

                if (successIndex) // we just reloc'd
                {
                    DETERMINISTIC_CHECK(m_impl->Determinator, 234525);

                    const KeyframeProxy& relativeKeyframe = candidates[*successIndex];
                    return std::make_shared<const PoseEstimated>(
                        std::move(currentKeyframe),
                        true,
                        relativeKeyframe.GetAnalyzedImage()->GetFrameId(),
                        relativeKeyframe.GetPose()
                    );
                }
            }

            return mira::errc::failed;
        });
    }

    PoseEstimationWorker::~PoseEstimationWorker() = default;
}
