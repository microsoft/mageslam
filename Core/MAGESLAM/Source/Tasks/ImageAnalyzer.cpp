// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "ImageAnalyzer.h"

#include "arcana/analysis/determinator.h"
#include "MageSettings.h"
#include "Utils/Logging.h"
#include "MageContext.h"
#include "MageSlam.h"
#include "Image/ImagePreprocessor.h"
#include "Utils/CameraConfiguration.h"

#include "Platform/Platform.h"

#include <arcana/timer.h>
#include <arcana/threading/task.h>
#include <gsl/gsl>

namespace mage
{
    namespace
    {
        //outImageCalibration: the calibration generated to match the outImage (can be distorted if input image is distorted and image pixels are not undistorted, otherwise is undistorted)
        //outUndistortedCalibration: always the undistorted calibration for the image
        void Undistort(const FrameData& frame, ImagePreprocessor& preProc, bool undistortImagePixels, CameraCalibration& outImageCalibration, CameraCalibration& outUndistortedCalibration, cv::Mat& outImage)
        {
            std::shared_ptr<AnalyzedImage> currentFrame{};

            if (frame.Format.CameraModel->GetDistortionType() == calibration::DistortionType::None)
            {
                //image comes in undistorted, pass through
                outImage = frame.ImageMat;
                outUndistortedCalibration = CameraCalibration(frame.Format.CameraModel);
                outImageCalibration = outUndistortedCalibration;
            }
            else if (undistortImagePixels)
            {
                //image comes in distorted, pixels should be undistorted
                CameraCalibration distortedCalibration(frame.Format.CameraModel);
                preProc.UndistortImage(frame.ImageMat, distortedCalibration, outImage, outUndistortedCalibration);
                outImageCalibration = outUndistortedCalibration;
            }
            else
            {
                //image comes in distorted, but leave pixels distorted and undistort keypoints
                outImage = frame.ImageMat;
                outImageCalibration = CameraCalibration(frame.Format.CameraModel);
                preProc.CalculateUndistortedCalibration({ (int)frame.ImageMat.cols, (int)frame.ImageMat.rows }, outImageCalibration, outUndistortedCalibration);
            }
        }
    }

    struct ImageAnalyzer::Impl
    {
        mira::determinator& Determinator;
        MageContext& Context;
        const MageSlamSettings& Settings;

        std::map<CameraIdentity, OrbFeatureDetector> Detectors;
        std::map<CameraIdentity, ImagePreprocessor> Preprocessors;

        // Max size 72 is required for x64 builds
        mira::background_dispatcher<72> MainDispatcher;

        // Max size 72 is required for x64 builds
        mira::background_dispatcher<72> BackgroundDispatcher;
        memory_pool BackgroundMemory{ 100 * 1024, 5 * 1000 * 1024 };
        
        std::vector<MAGESlam::CameraConfiguration> Configurations;

        Impl(mira::determinator& determinator, MageContext& context, const MageSlamSettings& settings, gsl::span<const MAGESlam::CameraConfiguration> configurations)
            : Determinator{ determinator }
            , Context{ context }
            , Settings{ settings }
            , Configurations{ configurations.begin(), configurations.end() }
        {
            MainDispatcher.queue([]() { mage::platform::set_thread_name("Mage ImageAnalyzer Main Thread"); });
            BackgroundDispatcher.queue([]() { mage::platform::set_thread_name("Mage ImageAnalyzer Background Thread"); });

            for (const auto& config : configurations)
            {
                Detectors.emplace(config.CameraIdentity, OrbFeatureDetector{ GetSettingsForCamera(settings, config.CameraIdentity).FeatureExtractorSettings });
                Preprocessors.emplace(config.CameraIdentity, ImagePreprocessor{});
            }
        }
    };
   
    ImageAnalyzer::ImageAnalyzer(mira::determinator& determinator, MageContext& context, gsl::span<const MAGESlam::CameraConfiguration> configurations, const MageSlamSettings& settings)
        : BaseWorker{ 100 * 1024, 5 * 1000 * 1024 }
        , m_impl{ std::make_unique<Impl>(determinator, context, settings, configurations) }
    {}

    mira::task<FrameAnalyzed> ImageAnalyzer::ProcessFrame(const std::shared_ptr<FrameData>& frame)
    {
        DETERMINISTIC_CHECK(m_impl->Determinator, frame->ImageData);

        if (frame->ImageData == nullptr)
        {
            return mira::task_from_error<FrameAnalyzed>(mira::errc::skipped);
        }

        return mira::make_task(m_impl->MainDispatcher, Cancellation(), [this, frame]()
        {
            SCOPE_TIMER(Threads::RunImageThread);

            thread_memory memory = MemoryPool().create();

            const auto preprocesor = m_impl->Preprocessors.find(frame->Format.FrameId.Camera);
            assert(preprocesor != m_impl->Preprocessors.end());

            const auto& cameraSettings = GetSettingsForCamera(m_impl->Settings, frame->Format.FrameId.Camera);

            CameraCalibration imageMatCalibration, undistortedCalibration;
            Undistort(*frame, preprocesor->second, cameraSettings.UndistortImagePixels, imageMatCalibration, undistortedCalibration, frame->ImageMat);

            const auto detector = m_impl->Detectors.find(frame->Format.FrameId.Camera);
            assert(detector != m_impl->Detectors.end());
            detector->second.Process(imageMatCalibration, undistortedCalibration, memory, frame->ImageData, frame->ImageMat);

            std::shared_ptr<AnalyzedImage> currentFrame = std::make_shared<AnalyzedImage>(std::move(frame->ImageData), frame->Format.FrameId, imageMatCalibration, undistortedCalibration, frame->Format.Timestamp, frame->ImageMat);

            DETERMINISTIC_CHECK(m_impl->Determinator, currentFrame->GetImageData());

            m_impl->Context.BagOfWords.AddTrainingDescriptors({ currentFrame->GetDescriptors().data(), (ptrdiff_t)currentFrame->GetDescriptors().size() });

            return FrameAnalyzed{ frame, currentFrame };
        });
    }

    mira::task<StereoFramesAnalyzed> ImageAnalyzer::ProcessFrames(const std::shared_ptr<FrameData>& frameOne, const std::shared_ptr<FrameData>& frameTwo)
    {
        DETERMINISTIC_CHECK(m_impl->Determinator, frameOne->ImageData);
        DETERMINISTIC_CHECK(m_impl->Determinator, frameTwo->ImageData);

        if (frameOne->ImageData == nullptr)
        {
            return mira::task_from_error<StereoFramesAnalyzed>(mira::errc::skipped);
        }

        assert(frameOne->ImageData->GetCameraIdentity() == CameraIdentity::STEREO_1 && "expecting camera one to be stereo 1");
        assert(frameTwo->ImageData->GetCameraIdentity() == CameraIdentity::STEREO_2 && "expecting camera two to be stereo 2");
        assert(frameTwo->ImageData != nullptr && "if we have an one frame, we should have two");

        return mira::make_task(m_impl->MainDispatcher, Cancellation(), [this, frameOne, frameTwo]()
        {
            SCOPE_TIMER(Threads::RunImageThread::Stereo);

            mira::task_completion_source<CameraCalibration> targetCalibration;

            if (frameOne->Format.CameraModel->GetDistortionType() == calibration::DistortionType::None)
            {
                targetCalibration.complete(CameraCalibration{ frameOne->Format.CameraModel });
            }

            //TODO: stop processing wide frames once stereo init is complete
            auto processOne = mira::make_task(m_impl->BackgroundDispatcher, mira::cancellation::none(), [targetCalibration, frameOne, this]() mutable
            {
                SCOPE_TIMER(Threads::RunImageThread::Stereo::Frame1);

                thread_memory memory = m_impl->BackgroundMemory.create();

                const auto preprocesor = m_impl->Preprocessors.find(frameOne->Format.FrameId.Camera);
                assert(preprocesor != m_impl->Preprocessors.end());

                const auto& cameraSettingsOne = GetSettingsForCamera(m_impl->Settings, frameOne->Format.FrameId.Camera);

                CameraCalibration imageMatCalibration, undistortedCalibration;
                Undistort(*frameOne, preprocesor->second, cameraSettingsOne.UndistortImagePixels, imageMatCalibration, undistortedCalibration, frameOne->ImageMat);

                const auto detector = m_impl->Detectors.find(frameOne->Format.FrameId.Camera);
                assert(detector != m_impl->Detectors.end());
                detector->second.Process(imageMatCalibration, undistortedCalibration, memory, frameOne->ImageData, frameOne->ImageMat);

                auto frame1 = std::make_shared<AnalyzedImage>(std::move(frameOne->ImageData), frameOne->Format.FrameId, imageMatCalibration, undistortedCalibration, frameOne->Format.Timestamp, frameOne->ImageMat);

                if (!targetCalibration.completed())
                {
                    targetCalibration.complete(undistortedCalibration);
                }

                m_impl->Context.BagOfWords.AddTrainingDescriptors({ frame1->GetDescriptors().data(), (ptrdiff_t)frame1->GetDescriptors().size() });

                return frame1;
            });

            auto processTwo = mira::make_task(mira::inline_scheduler, mira::cancellation::none(), [frameTwo, targetCalibration, this]
            {
                SCOPE_TIMER(Threads::RunImageThread::Stereo::Frame2);

                const auto preprocesor = m_impl->Preprocessors.find(frameTwo->Format.FrameId.Camera);
                assert(preprocesor != m_impl->Preprocessors.end());

                const auto& cameraSettingsTwo = GetSettingsForCamera(m_impl->Settings, frameTwo->Format.FrameId.Camera);

                CameraCalibration imageMatCalibration, undistortedCalibration;
                Undistort(*frameTwo, preprocesor->second, cameraSettingsTwo.UndistortImagePixels, imageMatCalibration, undistortedCalibration, frameTwo->ImageMat);

                return targetCalibration.as_task().then(m_impl->MainDispatcher, mira::cancellation::none(),
                    [imageMatCalibration, undistortedCalibration, preprocesor, frameTwo, this](const CameraCalibration& targetCalibration)
                    {
                        MAGESlam::CameraConfiguration sourceConfig = mage::FindCameraConfiguration(CameraIdentity::STEREO_2, m_impl->Configurations);
                        MAGESlam::CameraConfiguration targetConfig = mage::FindCameraConfiguration(CameraIdentity::STEREO_1, m_impl->Configurations);  //TODO: experiment: scale up wide to narrow when narrow will be primary tracked after init?

                        thread_memory memory = MemoryPool().create();

                        float scaleSourceToTarget{};
                        CameraCalibration scaledCameraCalibrationUndistorted;
                        bool result = preprocesor->second.ScaleImageForCameraConfiguration(sourceConfig, undistortedCalibration, targetConfig, targetCalibration, frameTwo->ImageMat, m_impl->Settings.StereoSettings, frameTwo->ImageMat, scaledCameraCalibrationUndistorted, scaleSourceToTarget);
                        assert(result && "camera preprocessing failed"); result;

                        CameraCalibration scaledCameraCalibrationDistorted = imageMatCalibration.GetScaledCalibration(scaleSourceToTarget);

                        const auto detector = m_impl->Detectors.find(CameraIdentity::STEREO_2);
                        assert(detector != m_impl->Detectors.end());
                        detector->second.Process(scaledCameraCalibrationDistorted, scaledCameraCalibrationUndistorted, memory, frameTwo->ImageData, frameTwo->ImageMat);

                        auto frame2 = std::make_shared<AnalyzedImage>(std::move(frameTwo->ImageData), frameTwo->Format.FrameId, scaledCameraCalibrationDistorted, scaledCameraCalibrationUndistorted, frameTwo->Format.Timestamp, frameTwo->ImageMat);

                        m_impl->Context.BagOfWords.AddTrainingDescriptors({ frame2->GetDescriptors().data(), (ptrdiff_t)frame2->GetDescriptors().size() });

                        return frame2;
                    });
            });

            return mira::when_all(processOne, processTwo).then(mira::inline_scheduler, mira::cancellation::none(),
                [this, frameOne, frameTwo](const std::tuple<std::shared_ptr<AnalyzedImage>, std::shared_ptr<AnalyzedImage>>& images) mutable
                {
                    StereoFramesAnalyzed framesAnalyzed;
                     
                    framesAnalyzed.One.SourceFrame = frameOne;
                    framesAnalyzed.One.Analyzed = std::get<0>(images);

                    framesAnalyzed.Two.SourceFrame = frameTwo;
                    framesAnalyzed.Two.Analyzed = std::get<1>(images);

                    return framesAnalyzed;
                });
        });
    }

    ImageAnalyzer::~ImageAnalyzer() = default;
}
