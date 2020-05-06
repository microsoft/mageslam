// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <iosfwd>
#include <gsl\gsl>
#include <vector>

#include "Data\Data.h"
#include "Data\TimeUnits.h"

#include <Plat/CameraDevice/CameraSettings.h>

namespace mage
{
    namespace binary_data
    {
        constexpr uint64_t LATEST_BINARY_VERSION = 3;
        constexpr uint64_t LATEST_CALIBRATION_VERSION = 2;
        
        // portable strings for device information
        struct DeviceInformation
        {
            std::string Id;
            std::string DeviceName;
            std::string DeviceManufacturer;
            std::string DeviceHardwareVersion;
            std::string DeviceFirmwareVersion;
        };

        struct HeaderData
        {
            uint64_t BinaryVersion;
            uint64_t CalibrationVersion;
            calibration::LinearFocalLengthModel Calibration;
            mage::Size ImageSize;
            mage::PixelFormat FormatOfPixels;

            DeviceInformation DeviceInformation;

            HeaderData(const uint64_t& b, const uint64_t& m, const calibration::LinearFocalLengthModel& c, const mage::Size& size, const mage::PixelFormat &fp, const mage::binary_data::DeviceInformation& deviceInfo)
                : BinaryVersion{ b }, CalibrationVersion{ m }, Calibration{ c }, ImageSize{ size }, FormatOfPixels{ fp }, DeviceInformation{ deviceInfo }
            {}

            HeaderData()
                : BinaryVersion{ LATEST_BINARY_VERSION },
                CalibrationVersion{ LATEST_CALIBRATION_VERSION },
                Calibration{},
                ImageSize{},
                FormatOfPixels{ mage::PixelFormat::GRAYSCALE8 },
                DeviceInformation{ "n/a", "n/a", "n/a", "n/a" }
            {}
        };
              
        struct FileFrameData
        {
            uint64_t CorrelationId;
            hundred_nanoseconds TimeStamp;
            mira::CameraSettings CameraSettings;
            std::vector<uint8_t> Pixels;

            FileFrameData(const uint64_t& correlationId, const hundred_nanoseconds& t, const mira::CameraSettings& cameraSettings, std::vector<uint8_t> p)
                : CorrelationId{ correlationId }, TimeStamp{ t }, CameraSettings{ cameraSettings }, Pixels{ std::move(p) }
            {}
            FileFrameData() = default;
            FileFrameData(FileFrameData&&) = default;
        };

        void SerializeCaptureHeaderData(std::fstream& outputFile, const HeaderData& data);
        void SerializeFrameData(std::fstream& outputFile, const FileFrameData& data);

        bool DeSerializeHeaderData(std::istream& inputFile, HeaderData& data);
        FileFrameData DeSerializeFrameData(std::istream& inputFile, HeaderData& headerData);
    }
}

