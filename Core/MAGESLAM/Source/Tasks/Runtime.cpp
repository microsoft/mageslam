// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Runtime.h"

#include "Data/FrameData.h"

#include <arcana/string.h>

#include "MageContext.h"

#include "ImageAnalyzer.h"
#include "InitializationWorker.h"
#include "StereoInitializationWorker.h"
#include "PoseEstimationWorker.h"
#include "TrackLocalMapWorker.h"
#include "MappingWorker.h"
#include "Fuserworker.h"
#include "LoopClosureWorker.h"
#include "MotionModelPriorProvider.h"
#include "IMUPosePriorProvider.h"

#include "Platform/Platform.h"

#include "Utils/MageConversions.h"
#include "Utils/Logging.h"

#include <arcana/analysis/object_trace.h>
#include <Analysis/DataPoints.h>

namespace mage
{
    struct Runtime::Impl : public BaseWorker
    {
        const MageSlamSettings& m_settings;
        MageContext& m_context;
        Fuser& m_fuser;
        mira::state_machine_driver& m_driver;

        // Max size 72 is required for x64 builds
        mira::background_dispatcher<72> m_runtimeDispatcher;
        mira::background_dispatcher<72> m_trackingDispatcher;

        TrackingMediator m_trackingMediator;
        mira::determinator& m_trackingDeterminator = mira::determinator::create("TrackingThread");

        // Max size 72 is required for x64 builds
        mira::background_dispatcher<72> m_mappingDispatcher;
        mira::determinator& m_mappingDeterminator = mira::determinator::create("MappingThread");

        std::vector<PoseRefined> m_pendingKeyframes;

        std::vector<MAGESlam::CameraConfiguration> m_cameraConfigurations;

        std::unique_ptr<ImageAnalyzer> m_imageAnalyzer;
        std::unique_ptr<InitializationWorker> m_initWorker;
        std::unique_ptr<StereoInitializationWorker> m_stereoInitWorker;

        // TODO: Fuser std::unique_ptr<ExternalCASIMU> m_externalCas;

        std::unique_ptr<MotionModelPriorProvider> m_motionModelProvider;
        std::unique_ptr<IMUPosePriorProvider> m_imuPriorProvider;
        // TODO: Fuser std::unique_ptr<DynamicIMUPosePriorProvider> m_dynamicPriorProvider;

        IPosePriorProvider* m_priorProvider = nullptr;

        std::unique_ptr<PoseEstimationWorker> m_poseEstimator;
        std::unique_ptr<TrackLocalMapWorker> m_trackLocalMapWorker;
        std::unique_ptr<MappingWorker> m_mappingWorker;
        std::unique_ptr<FuserWorker> m_fuserWorker;
        std::unique_ptr<LoopClosureWorker> m_loopClosureWorker;

        std::vector<IIMUReceiver*> m_imuReceivers;

        struct
        {
            std::atomic<bool> Pending = false;
        } AnalysisData = {};

        struct
        {
            bool Initialized = false;
            bool IsLost = false;
            size_t LostCount = 0;

            std::atomic<bool> Pending = false;
        } TrackingData = {};

        Impl(
            const MageSlamSettings& settings,
            MageContext& context,
            Fuser& fuser,
            mira::state_machine_driver& driver)
            :   BaseWorker{ 100 * 1024, 10 * 1000 * 1024 },
                m_settings{ settings },
                m_context{ context },
                m_fuser{ fuser },
                m_driver{ driver },
                m_trackingMediator{ m_trackingDispatcher }
        {
            m_runtimeDispatcher.queue([]() { mage::platform::set_thread_name("Mage Runtime Thread"); });
            m_trackingDispatcher.queue([]() { mage::platform::set_thread_name("Mage Tracking Thread"); });
            m_mappingDispatcher.queue([]() { mage::platform::set_thread_name("Mage Mapping Thread"); });
        }

        template<typename ResultT, typename FactoryT>
        mira::task<ResultT> OneAtATime(std::atomic<bool>& pending, FactoryT&& factory)
        {
            bool expected = false;
            if (!pending.compare_exchange_strong(expected, true))
            {
                return mira::task_from_error<ResultT>(mira::errc::skipped);
            }

            return factory().then(mira::inline_scheduler, mira::cancellation::none(),
                [&pending](const mira::expected<ResultT>& result)
            {
                pending = false;
                return result;
            });
        }

        mira::task<PoseRefined> TryToMonocularInit(const FrameAnalyzed& analyzed)
        {
            return m_initWorker->Initialize(analyzed)
                .then(m_runtimeDispatcher, Cancellation(), [this](const mira::expected<PoseRefined>& refined)
                {
                    if (!refined)
                        return mira::task_from_error<PoseRefined>(refined.error());

                    return m_initWorker->DisposeAsync()
                        .then(mira::inline_scheduler, Cancellation(), [this, refined]()
                        {
                            m_initWorker = nullptr;

                            return refined;
                        });
                });
        }

        mira::task<PoseRefined> TryToStereoInit(const StereoFramesAnalyzed& analyzed)
        {
            if (m_stereoInitWorker == nullptr)
            {
                auto& trackingFrame = m_settings.StereoSettings.PrimaryTrackingCamera == CameraIdentity::STEREO_1 ? analyzed.One : analyzed.Two;
                return TryToMonocularInit(trackingFrame);
            }

            return m_stereoInitWorker->Initialize(analyzed)
                .then(m_runtimeDispatcher, Cancellation(), [this](const mira::expected<PoseRefined>& refined)
                {
                    if (!refined)
                        return mira::task_from_error<PoseRefined>(refined.error());

                    return m_stereoInitWorker->DisposeAsync()
                        .then(mira::inline_scheduler, Cancellation(), [this, refined]()
                        {
                            m_stereoInitWorker = nullptr;

                            return refined;
                        });
                });
        }

        mira::task<PoseRefined> TrackFrame(const FrameAnalyzed& analyzed)
        {
            mira::task<std::shared_ptr<const PoseEstimated>> estimationTask;

            if (TrackingData.IsLost)
            {
                m_priorProvider->OnTrackingLost();

                estimationTask = m_poseEstimator->EstimatePose(analyzed, m_trackLocalMapWorker->GetHistory(), boost::none);
            }
            else
            {
                estimationTask = m_priorProvider->GetPoseForTime(m_trackLocalMapWorker->GetHistory(), analyzed.Analyzed->GetTimeStamp())
                    .then(m_runtimeDispatcher, Cancellation(), [analyzed, this](const Pose& pose)
                    {
                        auto estimated = m_poseEstimator->EstimatePose(analyzed, m_trackLocalMapWorker->GetHistory(), pose);

#ifdef OBJECT_TRACE_ENABLED
                        estimated.then(mira::inline_scheduler, mira::cancellation::none(), [pose](const std::shared_ptr<const PoseEstimated>& estimated)
                        {
                            FIRE_OBJECT_TRACE("PoseEstimationTranslation.Value", nullptr, (make_frame_data_point(
                                estimated->Frame->GetAnalyzedImage()->GetFrameId(),
                                estimated->Frame->GetAnalyzedImage()->GetTimeStamp(),
                                cv::norm(estimated->Frame->GetPose().GetWorldSpacePosition() - pose.GetWorldSpacePosition()))));

                            FIRE_OBJECT_TRACE("PoseEstimationRotation.Radians", nullptr, (make_frame_data_point(
                                estimated->Frame->GetAnalyzedImage()->GetFrameId(),
                                estimated->Frame->GetAnalyzedImage()->GetTimeStamp(),
                                (double)estimated->Frame->GetPose().GetRotationQuaternion().angularDistance(pose.GetRotationQuaternion()))));
                        });
#endif

                        return estimated;
                    });
            }

            return estimationTask.then(m_runtimeDispatcher, Cancellation(), [this](const std::shared_ptr<const PoseEstimated>& estimated)
                {
                    m_trackingMediator.send(estimated);

                    // we have a valid tracking result
                    TrackingData.LostCount = 0;
                    TrackingData.IsLost = false;

                    m_context.Introspection.IntrospectEstimatedPose(estimated->Frame->GetAnalyzedImage()->GetFrameId(), mage::ToMageMat(estimated->Frame->GetPose().GetViewMatrix()));

                    auto refined = m_trackLocalMapWorker->RefinePose(estimated);

#ifdef OBJECT_TRACE_ENABLED
                    auto estimatedPose = estimated->Frame->GetPose();

                    refined.then(mira::inline_scheduler, mira::cancellation::none(), [estimatedPose](const PoseRefined& refined)
                    {
                        FIRE_OBJECT_TRACE("PoseRefinementTranslation.Value", nullptr, (make_frame_data_point(
                            refined.Frame->GetAnalyzedImage()->GetFrameId(),
                            refined.Frame->GetAnalyzedImage()->GetTimeStamp(),
                            cv::norm(refined.Frame->GetPose().GetWorldSpacePosition() - estimatedPose.GetWorldSpacePosition()))));

                        FIRE_OBJECT_TRACE("PoseRefinementRotation.Radians", nullptr, (make_frame_data_point(
                            refined.Frame->GetAnalyzedImage()->GetFrameId(),
                            refined.Frame->GetAnalyzedImage()->GetTimeStamp(),
                            (double)refined.Frame->GetPose().GetRotationQuaternion().angularDistance(estimatedPose.GetRotationQuaternion()))));
                    });
#endif

                    return refined;
                });
        }

        mira::task<PoseRefined> TrackFrames(const StereoFramesAnalyzed& analyzed)
        {
            auto& trackingFrame = m_settings.StereoSettings.PrimaryTrackingCamera == CameraIdentity::STEREO_1 ? analyzed.One : analyzed.Two;
            return TrackFrame(trackingFrame);
        }

        mira::task<FrameAnalyzed> AnalyzeFrame(const std::shared_ptr<FrameData>& frame)
        {
            return m_imageAnalyzer->ProcessFrame(frame)
                .then(mira::inline_scheduler, mira::cancellation::none(), [this](const mira::expected<FrameAnalyzed>& analyzed) -> mira::expected<FrameAnalyzed>
                {
                    m_context.Introspection.IntrospectAnalyzedImage(*analyzed->SourceFrame, *analyzed->Analyzed);

                    if (!m_context.BagOfWords.IsTrainingDone())
                    {
                        return mira::errc::skipped;
                    }
                    return analyzed;
                });
        }

        mira::task<StereoFramesAnalyzed> AnalyzeFrames(const std::shared_ptr<FrameData>& one, const std::shared_ptr<FrameData>& two)
        {
            return m_imageAnalyzer->ProcessFrames(one, two)
                .then(mira::inline_scheduler, mira::cancellation::none(), [this](const mira::expected<StereoFramesAnalyzed>& stereoAnalyzed) -> mira::expected<StereoFramesAnalyzed>
                {
                    m_context.Introspection.IntrospectAnalyzedImage(*stereoAnalyzed->One.SourceFrame, *stereoAnalyzed->One.Analyzed);
                    m_context.Introspection.IntrospectAnalyzedImage(*stereoAnalyzed->Two.SourceFrame, *stereoAnalyzed->Two.Analyzed);
                    
                    if (!m_context.BagOfWords.IsTrainingDone())
                    {
                        return mira::errc::skipped;
                    }
                    return stereoAnalyzed;
                });
        }

        void FinalizeFrame(FrameData& frame, const mira::expected<PoseRefined>& result)
        {
            if (result)
            {
                m_trackingMediator.send(*result);

                frame.Set({ mage::ToMageMat(result->Frame->GetPose().GetViewMatrix()), TrackingState::TRACKING });
            }
            else if (result.error() == mira::errc::skipped)
            {
                frame.Set({ mage::CreateIdentityMageMatrix(), TrackingState::SKIPPED });
            }
            else if (result.error() == mira::errc::not_enough_input)
            {
                frame.Set({ mage::CreateIdentityMageMatrix(), TrackingState::INITIALIZING });
            }
            else
            {
                // we either failed pose estimation or track local map
                frame.Set({ mage::CreateIdentityMageMatrix(), TrackingState::RELOCALIZING });

                TrackingData.LostCount++;
                if (!TrackingData.IsLost && TrackingData.LostCount > m_settings.TrackLocalMapSettings.TrackingLostCountUntilReloc)
                {
                    LogMessage<>(L"Runtime -> Tracking Lost");

                    TrackingData.IsLost = true;
                    m_trackingMediator.send(TrackingLost{});
                }
            }
        }

        void CompleteInitialization()
        {
            TrackingData.Initialized = true;
            m_trackingMediator.send(InitCompleted{});

            auto keyframes = m_context.Map.GetAllKeyframes();

            if (m_settings.StereoSettings.UseStereoInit)
            {
                keyframes.erase(std::remove_if(keyframes.begin(), keyframes.end(), [this](const KeyframeProxy& proxy)
                {
                    return m_settings.StereoSettings.PrimaryTrackingCamera != proxy.GetAnalyzedImage()->GetFrameId().Camera;
                }), keyframes.end());
            }

            m_poseEstimator = std::make_unique<PoseEstimationWorker>(
                m_trackingMediator, m_trackingDeterminator, m_fuser, m_context, m_settings);

            m_trackLocalMapWorker = std::make_unique<TrackLocalMapWorker>(
                keyframes, m_trackingMediator, m_trackingDeterminator, m_context, m_settings);

            if (m_settings.StereoSettings.UseStereoInit)
            {
                m_mappingWorker = std::make_unique<MappingWorker>(
                    m_mappingDeterminator, m_context, m_settings, GetSettingsForCamera(m_settings, m_settings.StereoSettings.PrimaryTrackingCamera));
            }
            else
            {
                m_mappingWorker = std::make_unique<MappingWorker>(
                    m_mappingDeterminator, m_context, m_settings, GetSettingsForCamera(m_settings, mage::CameraIdentity::MONO));
            }

            m_loopClosureWorker = std::make_unique<LoopClosureWorker>(m_context, m_settings);

            Pending() += TrackingSchedule().then(mira::inline_scheduler, mira::cancellation::none(), [](const mira::expected<void>& result)
            {
                assert(!result && "Tracking schedule should never complete unless its cancelled");

                std::wstringstream ss;
                if (result)
                {
                    ss << "Tracking Completed Without Error" << std::endl;
                }
                else
                {
                    ss << "Tracking Completed With: " << mira::utf8_to_utf16(result.error().message()) << std::endl;
                }

                LogMessage<>(ss.str());
            });
        }

        void EnqueNewKeyframe(const PoseRefined& poseRefined)
        {
            m_pendingKeyframes.emplace_back(poseRefined);
            if (m_pendingKeyframes.size() > m_settings.MappingSettings.MaxPendingKeyframes)
            {
                m_pendingKeyframes.erase(m_pendingKeyframes.begin());
            }
            m_mappingWorker->SetMappingWorkAvailable(!m_pendingKeyframes.empty());
        }

        void TrackMono(const std::shared_ptr<FrameData>& frame)
        {
            Pending() += OneAtATime<FrameAnalyzed>(AnalysisData.Pending, [&] { return AnalyzeFrame(frame); })
                .then(m_runtimeDispatcher, Cancellation(), [this](const FrameAnalyzed& analyzed)
                {
                    return OneAtATime<PoseRefined>(TrackingData.Pending, [&]
                    {
                        m_trackingMediator.send(AnalysisCompleted{ analyzed.Analyzed });

                        if (!TrackingData.Initialized)
                        {
                            return TryToMonocularInit(analyzed).then(m_runtimeDispatcher, Cancellation(), [this](const PoseRefined& refined)
                            {
                                // on success start tracking
                                CompleteInitialization();

                                return refined;
                            });
                        }
                        else
                        {
                            return TrackFrame(analyzed);
                        }
                    });
                }).then(m_runtimeDispatcher, mira::cancellation::none(), [this, frame](const mira::expected<PoseRefined>& result)
                {
                    FinalizeFrame(*frame, result);

                    if (result && result->IsNewKeyframe)
                    {
                        EnqueNewKeyframe(*result);
                    }
                });
        }

        void TrackStereo(const std::shared_ptr<FrameData>& one, const std::shared_ptr<FrameData>& two)
        {
            Pending() += OneAtATime<StereoFramesAnalyzed>(AnalysisData.Pending, [&] { return AnalyzeFrames(one, two); })
                .then(m_runtimeDispatcher, Cancellation(), [this](const StereoFramesAnalyzed& analyzed)
                {
                    return OneAtATime<PoseRefined>(TrackingData.Pending, [&]
                    {
                        auto& trackingAnalyzed = m_settings.StereoSettings.PrimaryTrackingCamera == CameraIdentity::STEREO_1 ? analyzed.One : analyzed.Two;

                        m_trackingMediator.send(AnalysisCompleted{ trackingAnalyzed.Analyzed });

                        if (!TrackingData.Initialized)
                        {
                            return TryToStereoInit(analyzed).then(m_runtimeDispatcher, Cancellation(), [this](const PoseRefined& refined)
                            {
                                // on success start tracking
                                CompleteInitialization();

                                return refined;
                            });
                        }
                        else
                        {
                            return TrackFrames(analyzed);
                        }
                    });
                }).then(m_runtimeDispatcher, mira::cancellation::none(), [this, one, two](const mira::expected<PoseRefined>& result)
                {
                    auto& trackingFrame = m_settings.StereoSettings.PrimaryTrackingCamera == CameraIdentity::STEREO_1 ? one : two;
                    auto& otherFrame = m_settings.StereoSettings.PrimaryTrackingCamera == CameraIdentity::STEREO_1 ? two : one;

                    FinalizeFrame(*trackingFrame, result);
                    FinalizeFrame(*otherFrame, mira::errc::skipped);
                    if (result && result->IsNewKeyframe)
                    {
                        EnqueNewKeyframe(*result);
                    }
                });
        }

        void Run(gsl::span<const MAGESlam::CameraConfiguration> cameras)
        {
            m_cameraConfigurations = std::vector<MAGESlam::CameraConfiguration>(cameras.begin(), cameras.end());

            m_imageAnalyzer = std::make_unique<ImageAnalyzer>(mira::determinator::create("ImageAnalyzer"), m_context, cameras, m_settings);

            m_fuserWorker = std::make_unique<FuserWorker>(m_trackingMediator, m_trackingDeterminator, m_fuser, m_context, m_settings);

            switch (m_settings.RuntimeSettings.PosePriorSettings.PosePrior)
            {
            case mage::PosePriorMethod::VISUAL_INERTIAL_FUSION:
            {
                LogMessage<>(L"Pose Prior: Visual Inertial");

                // TODO: Fuser
                /*m_externalCas = std::make_unique<ExternalCASIMU>(
                    m_context.IMUCharacterization,
                    m_settings.RuntimeSettings.PosePriorSettings.AssumeIMUAndCameraAreAtSamePosition);

                m_imuPriorProvider = std::make_unique<IMUPosePriorProvider>(m_externalCas.get());*/
                m_imuReceivers.push_back(m_imuPriorProvider.get());

                m_priorProvider = m_imuPriorProvider.get();
                break;
            }
            case mage::PosePriorMethod::VISUAL_INERTIAL_FUSION_WITH_3DOF:
            {
                LogMessage<>(L"Pose Prior: Visual Inertial With 3Dof");

                /* TODO: Fuser
                m_dynamicPriorProvider = std::make_unique<DynamicIMUPosePriorProvider>(
                    m_context.IMUCharacterization,
                    m_settings.RuntimeSettings.PosePriorSettings.AssumeIMUAndCameraAreAtSamePosition);

                m_imuReceivers.push_back(m_dynamicPriorProvider.get());

                m_priorProvider = m_dynamicPriorProvider.get();*/
                break;
            }
            case mage::PosePriorMethod::MOTION_MODEL:
            {
                LogMessage<>(L"Pose Prior: Motion Model");

                m_motionModelProvider = std::make_unique<MotionModelPriorProvider>();

                m_priorProvider = m_motionModelProvider.get();
                break;
            }
            }

            if (cameras.size() == 2 && m_settings.StereoSettings.UseStereoInit)
            {
                m_stereoInitWorker = std::make_unique<StereoInitializationWorker>(
                    m_trackingDispatcher, m_trackingDeterminator, m_cameraConfigurations, m_context, m_settings);
            }
            else
            {
                m_initWorker = std::make_unique<InitializationWorker>(
                    m_trackingDispatcher, m_trackingDeterminator, m_context, m_settings);
            }
        }

        void AddSample(const mage::SensorSample& sample)
        {
            for (auto consumer : m_imuReceivers)
            {
                consumer->AddSample(sample);
            }
        }

        mira::task<void> TrackingSchedule()
        {
            return m_driver.move_to(TrackingReadState, Cancellation())
                .then(m_runtimeDispatcher, Cancellation(), [&](bool)
                {
                    if (!m_pendingKeyframes.empty())
                        return TrackingAndMappingSchedule();
                    else
                        return TrackingSchedule();
                });
        }

        mira::task<void> TrackingAndMappingSchedule()
        {
            return mira::make_task(m_runtimeDispatcher, Cancellation(), [&]
            {
                assert(!m_pendingKeyframes.empty());
                m_mappingWorker->MappingTask(m_pendingKeyframes.front()).then(mira::inline_scheduler, Cancellation(), [this](const Id<Keyframe>& kfId)
                {
                    return m_loopClosureWorker->AttemptLoopClosure(kfId);
                });
                m_pendingKeyframes.erase(m_pendingKeyframes.begin());
                m_mappingWorker->SetMappingWorkAvailable(!m_pendingKeyframes.empty());

                return m_driver.move_to(KeyframeInsertionAndMapPointCullingState, Cancellation());
            }).then(mira::inline_scheduler, Cancellation(), [&]
            {
                return m_driver.move_to(MapPointCreationState, Cancellation());
            }).then(mira::inline_scheduler, Cancellation(), [&]
            {
                return BundleAdjustSchedule(false, true);
            }).then(mira::inline_scheduler, Cancellation(), [&]
            {
                return m_driver.move_to(KeyframeCullingState, Cancellation());
            }).then(mira::inline_scheduler, Cancellation(), [&]
            {
                return TrackingRead(m_settings.RuntimeSettings.TrackingReadsPerLoopDetection);
            }).then(mira::inline_scheduler, Cancellation(), [&]
            {
                return m_driver.move_to(LoopDetectionState, Cancellation());
            }).then(mira::inline_scheduler, Cancellation(), [&](bool loopDetected)
            {
                if (loopDetected)
                {
                    return m_driver.move_to(StartLoopClosureState, Cancellation())
                        .then(m_runtimeDispatcher, Cancellation(), [&](const LoopClosureTrackingUpdate& trackingUpdate)
                    {
                        m_trackLocalMapWorker->ConsumeUpdateFromLoopClosure(trackingUpdate);
                        return TrackingRead(m_settings.RuntimeSettings.TrackingReadsPerLoopClosure);
                    }).then(mira::inline_scheduler, Cancellation(), [&]
                    {
                        return m_driver.move_to(EndLoopClosureState, Cancellation());
                    }).then(m_runtimeDispatcher, Cancellation(), [&]
                    {
                        m_pendingKeyframes.clear();
                        m_mappingWorker->SetMappingWorkAvailable(false);
                    });
                }
                else
                {
                    return mira::task_from_result();
                }
            }).then(m_runtimeDispatcher, Cancellation(), [&]
            {
                if (m_pendingKeyframes.empty())
                {
                    m_trackLocalMapWorker->SetMappingIdle();
                    return TrackingSchedule();
                }
                else
                {
                    return m_driver.move_to(TrackingReadState, Cancellation()).then(mira::inline_scheduler, Cancellation(), [this](bool)
                    {
                        return TrackingAndMappingSchedule();
                    });
                }
            });
        }

        mira::task<void> TrackingRead(size_t times)
        {
            if (times == 0)
                return mira::task_from_result();

            return m_driver.move_to(TrackingReadState, Cancellation()).
                then(mira::inline_scheduler, Cancellation(), [times, this](bool /*keyframeInserted*/)
                {
                    return TrackingRead(times - 1);
                });
        }

        mira::task<void> BundleAdjustSchedule(bool iterateAgain, bool first)
        {
            if (first)
            {
                return TrackingRead(2).then(mira::inline_scheduler, Cancellation(), [=]
                {
                    return m_driver.move_to(BundleAdjustFirstWriteToMapState, Cancellation())
                        .then(mira::inline_scheduler, Cancellation(), [this](bool iterateAgain)
                        {
                            return BundleAdjustSchedule(iterateAgain, false);
                        });
                });
            }
            else if (iterateAgain)
            {
                return TrackingRead(2).then(mira::inline_scheduler, Cancellation(), [=]
                {
                    return m_driver.move_to(BundleAdjustNthWriteToMapState, Cancellation())
                        .then(mira::inline_scheduler, Cancellation(), [this](bool iterateAgain)
                        {
                            return BundleAdjustSchedule(iterateAgain, false);
                        });
                });
            }
            else
            {
                return mira::task_from_result();
            }
        }

        virtual mira::task<void> OnDisposeAsync() override
        {
            // once everything is done, we clean up all our workers.
            std::vector<mira::task<void>> disposables;

            if (m_imageAnalyzer)
            {
                disposables.emplace_back(m_imageAnalyzer->DisposeAsync());
            }

            if (m_initWorker)
            {
                disposables.emplace_back(m_initWorker->DisposeAsync());
            }

            if (m_stereoInitWorker)
            {
                disposables.emplace_back(m_stereoInitWorker->DisposeAsync());
            }

            if (m_motionModelProvider)
            {
                disposables.emplace_back(m_motionModelProvider->DisposeAsync());
            }

            if (m_imuPriorProvider)
            {
                disposables.emplace_back(m_imuPriorProvider->DisposeAsync());
            }

            if (m_poseEstimator)
            {
                disposables.emplace_back(m_poseEstimator->DisposeAsync());
            }

            if (m_trackLocalMapWorker)
            {
                disposables.emplace_back(m_trackLocalMapWorker->DisposeAsync());
            }

            if (m_mappingWorker)
            {
                disposables.emplace_back(m_mappingWorker->DisposeAsync());
            }

            if (m_loopClosureWorker)
            {
                disposables.emplace_back(m_loopClosureWorker->DisposeAsync());
            }

            return mira::when_all(disposables)
                .then(m_runtimeDispatcher, mira::cancellation::none(), [&](const mira::expected<void>&)
                {
                    m_imageAnalyzer.reset();
                    m_initWorker.reset();
                    m_stereoInitWorker.reset();
                    m_poseEstimator.reset();
                    m_trackLocalMapWorker.reset();
                    m_mappingWorker.reset();
                    m_loopClosureWorker.reset();
                });
        }
    };

    Runtime::Runtime(const MageSlamSettings& settings, MageContext& context, Fuser& fuser, mira::state_machine_driver& driver)
        :   m_impl{std::make_unique<Impl>(settings, context, fuser, driver)}
    {}

    Runtime::~Runtime()
    {
        std::promise<void> prom;

        m_impl->DisposeAsync().then(mira::inline_scheduler, mira::cancellation::none(), [&]
        {
            prom.set_value();
        });

        prom.get_future().get();
    }

    void Runtime::Run(gsl::span<const MAGESlam::CameraConfiguration> cameras)
    {
        m_impl->Run(cameras);
    }

    void Runtime::AddSample(const mage::SensorSample& sample)
    {
        m_impl->AddSample(sample);
    }

    void Runtime::TrackMono(std::shared_ptr<FrameData> frame)
    {
        m_impl->TrackMono(std::move(frame));
    }

    void Runtime::TrackStereo(std::shared_ptr<FrameData> one, std::shared_ptr<FrameData> two)
    {
        m_impl->TrackStereo(std::move(one), std::move(two));
    }

}
