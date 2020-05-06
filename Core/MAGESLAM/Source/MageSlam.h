// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <future>
#include <array>
#include <gsl\gsl>

#include <boost\optional.hpp>

#include "Data\Data.h"
#include "MageSettings.h"
#include "Device\Device.h"

#include <SensorSample.h>
#include <Plat/CameraDevice/CameraSettings.h>

namespace mage
{
    class PoseHistory;

    /* Entry point of the MAGESlam library */
    class MAGESlam
    {
        struct Impl;
    public:

        struct CameraConfiguration
        {
            CameraIdentity CameraIdentity;
            Size Size{};
            PixelFormat Format;
            Matrix Extrinsics;

            CameraConfiguration(mage::CameraIdentity cameraIdentity, const mage::Size& size, PixelFormat format, const Matrix& extrinsics)
                : CameraIdentity{ cameraIdentity },
                Format{ format },
                Size{ size },
                Extrinsics{ extrinsics }
            {
            };
        };

        struct FrameFormat
        {
            const mage::FrameId FrameId;
            std::shared_ptr<const calibration::CameraModel> CameraModel;
            std::chrono::system_clock::time_point Timestamp;
            mira::CameraSettings CameraSettings;

            FrameFormat(
                const mage::FrameId& frameId,
                std::shared_ptr<const calibration::CameraModel> cameraModel,
                const std::chrono::system_clock::time_point& timestamp,                
                const mira::CameraSettings& cameraSettings)
                : FrameId{ frameId },
                CameraModel{ cameraModel },
                Timestamp{ timestamp },
                CameraSettings{ cameraSettings }
            {}
        };

        struct Tracking
        {
            /*
                The Matrix is a row-major column-vector view matrix. right handed
                    R, R, R, Tx
                    R, R, R, Ty
                    R, R, R, Tz
                    0, 0, 0, 1
            */
            Matrix Pose;
            TrackingState State;

            Tracking(const Matrix& mat, TrackingState state)
                : Pose{ mat }, State{ state }
            {}

            Tracking()
                : Pose{ }, State{ TrackingState::SKIPPED }
            {}

            bool IsPoseGood() const
            {
                return State == TrackingState::TRACKING;
            }

            bool IsPoseSkipped() const
            {
                return State == TrackingState::SKIPPED;
            }
        };

        struct TrackedFrame
        {
            Tracking Tracking;
            std::shared_ptr<const calibration::CameraModel> CameraModel;
            Depth Depth;

            TrackedFrame(const MAGESlam::Tracking& tracking, std::shared_ptr<const calibration::CameraModel> cameraModel, const mage::Depth& depth)
                : Tracking{ tracking }, CameraModel{ cameraModel }, Depth{ depth }
            {}

            TrackedFrame() = default;
        };

        class FossilizedMap
        {
        public:
            ~FossilizedMap();

            /*
                Get the most up-to-date pose for the specified frame ID
            */
            std::vector<boost::optional<MAGESlam::TrackedFrame>> GetTrackingResultsForFrames(const gsl::span<const FrameId> frameIds) const;

            bool TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest) const;

        private:
            FossilizedMap(std::unique_ptr<const PoseHistory> history, std::vector<Position> mapPoints, const MageSlamSettings& settings);

            friend class MAGESlam;

            struct Impl;
            std::unique_ptr<Impl> m_impl;
        };

        explicit MAGESlam(const MageSlamSettings&, gsl::span<const CameraConfiguration> cameras, const device::IMUCharacterization& imu);

        MAGESlam(const MAGESlam&) = delete;
        MAGESlam& operator =(const MAGESlam&) = delete;

        struct Frame
        {
            FrameFormat Format;
            gsl::span<const uint8_t> Bytes;

            Frame(const FrameFormat& format, gsl::span<const uint8_t> bytes)
                : Format(format),
                Bytes(bytes)
            {}
        };

        /*
            Processes an image frame and returns a future Pose and the state of the
            tracker while it was generating it.
        */
        std::future<Tracking> ProcessFrame(const Frame& frame);

        /*
            Processes an stereo image pair and returns a future Pose for each frame and the state of the
            tracker while it was generating it.
        */
        std::pair<std::future<Tracking>, std::future<Tracking>> ProcessStereoFrames(const Frame& one, const Frame& two);

        /*
            Get the most up-to-date pose for the specified frame ID
        */
        std::vector<boost::optional<MAGESlam::TrackedFrame>> GetTrackingResultsForFrames(const gsl::span<const FrameId> frameIds) const;

        // adds a sensor sample to the sample queue for sensor fusion
        void AddSensorSample(const SensorSample& sample);

        // get normalized vector in mage's world coordinate frame describing the fuser's gravity estimate
        // returns whether the gravity estimate is valid.
        bool GetGravityDirection(Direction& gravDir) const;

        // gets the scale to apply to mage to bring it to meters as estimated by
        // the stereo tracking code
        float GetStereoMageMeterEstimate() const;

        // gets the scale to apply to mage to bring it to meters as estimated by the fuser
        // returns whether the scale is valid
        bool  GetScaleFromIMU(float& scaleMAGEtoMeters) const;

        bool TryGetVolumeOfInterest(AxisAlignedVolume& volumeOfInterest) const;

        // Destroys the current instance of MAGESlam and converts it to a fossilized map
        // that can be used for offline queries of frame poses
        static std::unique_ptr<FossilizedMap> Fossilize(std::unique_ptr<MAGESlam> slam);

        ~MAGESlam();
    private:
        std::unique_ptr<Impl> m_impl;
    };
}
