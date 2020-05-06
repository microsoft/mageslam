// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "BinarySerializer.h"

#include "Map\ThreadSafeMap.h"
#include "Serialization\binary_magestream.h"

#include <fstream>
#include <chrono>

using namespace std;

constexpr char MAGE_STAMP[] = "MAGE";
constexpr int MAGE_STAMP_LENGTH = 4;

namespace mage
{
    namespace binary_data
    {
        namespace
        {
            std::string DeSerializeString(binary_magestream<std::istream>& stream)
            {
                uint32_t strLength{};
                stream.read<uint32_t>(strLength);
                std::vector<char> charBuffer(strLength);
                stream.read(charBuffer.data(), strLength);
                return std::string(charBuffer.begin(), charBuffer.end());
            }

            void SerializeString(binary_magestream<std::ostream>& serializer, const std::string& string)
            {
                serializer.write(gsl::narrow_cast<uint32_t>(string.length()));
                serializer.write(string.c_str(), string.length());
            }

            float GetFloatFromDouble(binary_magestream<std::istream>& deSerializer)
            {
                double temp;
                deSerializer.read(temp);

                return (float)temp;
            }

            void DeSerializeHeaderCalibrationDataV0(binary_magestream<std::istream>& deSerializer, calibration::LinearFocalLengthModel& calibration)
            {
                float Cx = GetFloatFromDouble(deSerializer);
                float Cy = GetFloatFromDouble(deSerializer);
                float FxM = GetFloatFromDouble(deSerializer);
                float FxB = GetFloatFromDouble(deSerializer);
                float FyM = GetFloatFromDouble(deSerializer);
                float FyB = GetFloatFromDouble(deSerializer);

                float K0 = GetFloatFromDouble(deSerializer);
                float K1 = GetFloatFromDouble(deSerializer);
                float K2 = GetFloatFromDouble(deSerializer);
                float P0 = GetFloatFromDouble(deSerializer);
                float P1 = GetFloatFromDouble(deSerializer);

                float FocalBoundsLower = GetFloatFromDouble(deSerializer);
                float FocalBoundsUpper = GetFloatFromDouble(deSerializer);
                float CalibrationSizeWidth = GetFloatFromDouble(deSerializer);
                float CalibrationSizeHeight = GetFloatFromDouble(deSerializer);

                calibration = calibration::LinearFocalLengthModel(
                    { FxM, FxB },                                           //fx M, B
                    { FyM, FyB },                                           //fy M, B
                    Cx,                                                     //cx
                    Cy,                                                     //cy
                    { FocalBoundsLower, FocalBoundsUpper },                 //focal bounds
                    { (size_t)CalibrationSizeWidth, (size_t)CalibrationSizeHeight },        //calibration size
                    { K0, K1, K2, P0, P1 });                                //poly3k
            }

            void DeSerializeHeaderCalibrationDataV1(binary_magestream<std::istream>& deSerializer, calibration::LinearFocalLengthModel& calibration)
            {
                float cx = GetFloatFromDouble(deSerializer);
                float cy = GetFloatFromDouble(deSerializer);
                float fxM = GetFloatFromDouble(deSerializer);
                float fxB = GetFloatFromDouble(deSerializer);
                float fyM = GetFloatFromDouble(deSerializer);
                float fyB = GetFloatFromDouble(deSerializer);

                std::vector<float> distortionPoly3k;
                bool anyPoly3k = false;
                for (int idx = 0; idx < 5; idx++)
                {
                    float curPoly3k = GetFloatFromDouble(deSerializer);
                    anyPoly3k |= curPoly3k != 0.0f;
                    distortionPoly3k.push_back(curPoly3k); //  K1, K2, K3, P1, P2
                }

                if (anyPoly3k)
                {
                    if (distortionPoly3k.size() != 5)
                    {
                        assert(false && "read incorrect number of distortion parameters from serialized data");
                        distortionPoly3k.clear();
                    }
                }
                else
                {
                    distortionPoly3k.clear();
                }

                std::vector<float> distortionRational6k;

                bool anyRational6k = false;
                for (int idx = 0; idx < 8; idx++)
                {
                    float curRational6k = GetFloatFromDouble(deSerializer);
                    anyRational6k |= curRational6k != 0.0f;
                    distortionRational6k.push_back(curRational6k); // K1, K2, K3, K4, K5, K6, P1, P2
                }

                if (anyRational6k)
                {
                    if (distortionRational6k.size() != 8)
                    {
                        assert(false && "read incorrect number of distortion parameters from serialized data");
                        distortionRational6k.clear();
                    }
                }
                else
                {
                    distortionRational6k.clear();
                }

                float focalBoundsLower = GetFloatFromDouble(deSerializer);
                float focalBoundsUpper = GetFloatFromDouble(deSerializer);
                size_t calibrationSizeWidth = (size_t)GetFloatFromDouble(deSerializer);
                size_t calibrationSizeHeight = (size_t)GetFloatFromDouble(deSerializer);

                calibration = { {fxM,fxB }, {fyM,fyB}, cx,  cy,
                {focalBoundsLower,focalBoundsUpper},
                {calibrationSizeWidth, calibrationSizeHeight},
                distortionPoly3k,
                distortionRational6k };

            }

            bool DeSerializeHeaderDataV0(binary_magestream<std::istream>& deSerializer, HeaderData& data)
            {
                uint32_t readWidth{}, readHeight{};
                deSerializer.read(readWidth);
                deSerializer.read(readHeight);
                data.ImageSize.Width = readWidth;
                data.ImageSize.Height = readHeight;

                deSerializer.read(data.CalibrationVersion);

                DeSerializeHeaderCalibrationDataV0(deSerializer, data.Calibration);

                data.FormatOfPixels = PixelFormat::GRAYSCALE8;

                return true;
            }

            void GetTimeStampAndUIDBasedOnVersion(binary_magestream<std::istream>& deSerializer, FileFrameData& data, const uint64_t& binaryVersion)
            {
                int64_t unsignedTimestampsInHNS;
                if (binaryVersion == 0)
                {
                    uint64_t unsignedTimestampInNanoseconds;
                    deSerializer.read(unsignedTimestampInNanoseconds);
                    unsignedTimestampsInHNS = (int64_t)(unsignedTimestampInNanoseconds / 100);
                    data.CorrelationId = unsignedTimestampInNanoseconds;
                }
                else
                {
                    deSerializer.read(data.CorrelationId);
                    deSerializer.read(unsignedTimestampsInHNS);
                }

                data.TimeStamp = hundred_nanoseconds{ unsignedTimestampsInHNS };
            }

            void DeSerializeCameraSettings(binary_magestream<std::istream>& deSerializer, mira::CameraSettings& cameraSettings)
            {
                mira::CameraSettings::Fields fields;
                uint32_t whiteBalance;
                mira::HundredsNanoseconds exposureTime;
                uint32_t lensPosition;
                uint32_t isoSpeed;
                float isoExternalGain;
                float isoDigitalGain;

                deSerializer.read<uint32_t>(fields.data);
                deSerializer.read<uint32_t>(whiteBalance);
                deSerializer.read<hundred_nanoseconds>(exposureTime);
                deSerializer.read<uint32_t>(lensPosition);
                deSerializer.read<uint32_t>(isoSpeed);
                deSerializer.read<float>(isoExternalGain);
                deSerializer.read<float>(isoDigitalGain);
                
                const auto& bits = fields.bits;
                cameraSettings = mira::CameraSettings{
                    bits.WhiteBalanceValid ? whiteBalance : boost::optional<uint32_t>{},
                    bits.ExposureTimeValid ? exposureTime : boost::optional<mira::HundredsNanoseconds>{},
                    bits.LensPositionValid ? lensPosition : boost::optional<uint32_t>{},
                    bits.IsoSpeedValid ? isoSpeed : boost::optional<uint32_t>{},
                    bits.IsoExternalGainValid ? isoExternalGain : boost::optional<float>{},
                    bits.IsoDigitalGainValid ? isoDigitalGain : boost::optional<float>{}
                };
            }

            bool DeSerializeHeaderDataV1(binary_magestream<std::istream>& deSerializer, HeaderData& data)
            {
                DeSerializeHeaderDataV0(deSerializer, data);

                deSerializer.read(data.FormatOfPixels);

                return true;
            }

            bool DeSerializeHeaderDataV2(binary_magestream<std::istream>& deSerializer, HeaderData& data)
            {
                data.DeviceInformation.Id = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceName = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceManufacturer = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceHardwareVersion = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceFirmwareVersion = DeSerializeString(deSerializer);

                return DeSerializeHeaderDataV1(deSerializer, data);
            }

            bool DeSerializeHeaderDataV3(binary_magestream<std::istream>& deSerializer, HeaderData& data)
            {
                data.DeviceInformation.Id = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceName = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceManufacturer = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceHardwareVersion = DeSerializeString(deSerializer);
                data.DeviceInformation.DeviceFirmwareVersion = DeSerializeString(deSerializer);

                uint32_t readWidth{}, readHeight{};
                deSerializer.read(readWidth);
                deSerializer.read(readHeight);
                data.ImageSize.Width = readWidth;
                data.ImageSize.Height = readHeight;

                deSerializer.read(data.CalibrationVersion);
                switch (data.CalibrationVersion)
                {
                case 1:
                    DeSerializeHeaderCalibrationDataV0(deSerializer, data.Calibration);
                    break;
                case 2:
                    DeSerializeHeaderCalibrationDataV1(deSerializer, data.Calibration);
                    break;
                default:
                    assert(false && "Unexpected calibrationVersion");
                    return false;
                }
                deSerializer.read(data.FormatOfPixels);
                return true;
            }

            FileFrameData DeSerializeFrameDataV0(binary_magestream<std::istream>& deSerializer, HeaderData& headerData)
            {
                mage::binary_data::FileFrameData frameData{};
                size_t bytesSize = headerData.ImageSize.Width*headerData.ImageSize.Height;
                assert(headerData.FormatOfPixels != mage::PixelFormat::NV12 && "not expecting compressed data in this version");
                frameData.Pixels.resize(bytesSize);

                GetTimeStampAndUIDBasedOnVersion(deSerializer, frameData, headerData.BinaryVersion);

                uint64_t focusValue;
                deSerializer.read(focusValue);

                frameData.CameraSettings.SetLensPosition(gsl::narrow_cast<int32_t>(focusValue));

                deSerializer.read(frameData.Pixels.data(), frameData.Pixels.size());

                return frameData;
            }

            FileFrameData DeSerializeFrameDataV1(binary_magestream<std::istream>& deSerializer, HeaderData& headerData)
            {
                mage::binary_data::FileFrameData frameData{};

                size_t bytesSize = headerData.ImageSize.Width*headerData.ImageSize.Height;
                if (headerData.FormatOfPixels == mage::PixelFormat::NV12)
                {
                    bytesSize += bytesSize / 2;
                }
                frameData.Pixels.resize(bytesSize);

                GetTimeStampAndUIDBasedOnVersion(deSerializer, frameData, headerData.BinaryVersion);

                uint64_t focusValue;
                deSerializer.read(focusValue);

                frameData.CameraSettings.SetLensPosition(gsl::narrow_cast<int32_t>(focusValue));

                deSerializer.read(frameData.Pixels.data(), frameData.Pixels.size());

                return frameData;
            }

            FileFrameData DeSerializeFrameDataV2(binary_magestream<std::istream>& deSerializer, HeaderData& headerData)
            {
                mage::binary_data::FileFrameData frameData{};

                size_t bytesSize = headerData.ImageSize.Width*headerData.ImageSize.Height;
                if (headerData.FormatOfPixels == mage::PixelFormat::NV12)
                {
                    bytesSize += bytesSize / 2;
                }
                frameData.Pixels.resize(bytesSize);

                GetTimeStampAndUIDBasedOnVersion(deSerializer, frameData, headerData.BinaryVersion);
                DeSerializeCameraSettings(deSerializer, frameData.CameraSettings);
                deSerializer.read(frameData.Pixels.data(), frameData.Pixels.size());

                return frameData;
            }

            FileFrameData DeSerializeFrameDataV3(binary_magestream<std::istream>& deSerializer, HeaderData& headerData)
            {
                //frame data unchanged
                return DeSerializeFrameDataV2(deSerializer, headerData);
            }
        }

        bool CheckForMageStamp(binary_magestream<std::istream>& deSerializer, std::istream& inputStream)
        {
            char mageStamp[MAGE_STAMP_LENGTH]{};
            deSerializer.read(mageStamp, MAGE_STAMP_LENGTH);
            if (strncmp(mageStamp, MAGE_STAMP, MAGE_STAMP_LENGTH) == 0)
            {
                return true;
            }

            // return the stream to its position before the read
            inputStream.seekg(-MAGE_STAMP_LENGTH, std::ios::cur);
            return false;
        }

        bool DeSerializeHeaderData(binary_magestream<std::istream>& deSerializer, HeaderData& data, const bool isMageFile)
        {
            deSerializer.read(data.BinaryVersion);

            // all binaries (except v0) should have a mage stamp
            if ((!isMageFile && data.BinaryVersion == 0) || (isMageFile && data.BinaryVersion > 0))
            {
                switch (data.BinaryVersion)
                {
                case 0:
                    return DeSerializeHeaderDataV0(deSerializer, data);
                case 1:
                    return DeSerializeHeaderDataV1(deSerializer, data);
                case 2:
                    return DeSerializeHeaderDataV2(deSerializer, data);
                case 3:
                    return DeSerializeHeaderDataV3(deSerializer, data);
                default:
                    assert(false && "unexpected header version");
                    return false;
                }
            }

            return false;
        }

        FileFrameData DeSerializeFrameData(binary_magestream<std::istream>& deSerializer, HeaderData& headerData)
        {
            switch (headerData.BinaryVersion)
            {
            case 0:
                return DeSerializeFrameDataV0(deSerializer, headerData);
            case 1:
                return DeSerializeFrameDataV1(deSerializer, headerData);
            case 2:
                return DeSerializeFrameDataV2(deSerializer, headerData);
            case 3:
                return DeSerializeFrameDataV3(deSerializer, headerData);
            default:
                assert(false && "unexpected binary version");
                return {};
            }
        }

        FileFrameData DeSerializeFrameData(std::istream& inputFile, HeaderData& headerData)
        {
            binary_magestream<std::istream> deSerializer{ inputFile };
            return DeSerializeFrameData(deSerializer, headerData);
        }

        bool DeSerializeHeaderData(istream& inputFile, HeaderData& data)
        {
            binary_magestream<std::istream> deSerializer{ inputFile };

            bool hasMageStamp = CheckForMageStamp(deSerializer, inputFile);
            bool isMageBin = DeSerializeHeaderData(deSerializer, data, hasMageStamp);
            return isMageBin;
        }

        void SerializeCaptureHeaderData(fstream& outputFile, const HeaderData& data)
        {
            binary_magestream<std::ostream> serializer{ outputFile };

            serializer.write(MAGE_STAMP, MAGE_STAMP_LENGTH);
            serializer.write(LATEST_BINARY_VERSION);
            SerializeString(serializer, data.DeviceInformation.Id);
            SerializeString(serializer, data.DeviceInformation.DeviceName);
            SerializeString(serializer, data.DeviceInformation.DeviceManufacturer);
            SerializeString(serializer, data.DeviceInformation.DeviceHardwareVersion);
            SerializeString(serializer, data.DeviceInformation.DeviceFirmwareVersion);
            serializer.write(gsl::narrow_cast<uint32_t>(data.ImageSize.Width));
            serializer.write(gsl::narrow_cast<uint32_t>(data.ImageSize.Height));
            serializer.write(data.CalibrationVersion);

            serializer.write<double>(data.Calibration.GetCx());
            serializer.write<double>(data.Calibration.GetCy());
            serializer.write<double>(data.Calibration.GetFx().M);
            serializer.write<double>(data.Calibration.GetFx().B);
            serializer.write<double>(data.Calibration.GetFy().M);
            serializer.write<double>(data.Calibration.GetFy().B);

            std::vector<float> distortionPoly3k = data.Calibration.GetDistortionPoly3k();
            if (!distortionPoly3k.empty())
            {
                assert(distortionPoly3k.size() == 5 && "unexpected number of terms in distortion");
                serializer.write<double>(distortionPoly3k[0]); // K1
                serializer.write<double>(distortionPoly3k[1]); // K2;
                serializer.write<double>(distortionPoly3k[2]); // K3;
                serializer.write<double>(distortionPoly3k[3]); // P1;
                serializer.write<double>(distortionPoly3k[4]); // P2;
            }
            else
            {
                for (int idx = 0; idx < 5; idx++)
                {
                    serializer.write<double>(0.0);
                }
            }

            std::vector<float> distortionRational6k = data.Calibration.GetDistortionRational6k();
            if (!distortionRational6k.empty())
            {
                assert(distortionRational6k.size() == 8 && "unexpected number of terms in distortion");
                serializer.write<double>(distortionRational6k[0]); // K1
                serializer.write<double>(distortionRational6k[1]); // K2;
                serializer.write<double>(distortionRational6k[2]); // K3;
                serializer.write<double>(distortionRational6k[3]); // K4;
                serializer.write<double>(distortionRational6k[4]); // K5;
                serializer.write<double>(distortionRational6k[5]); // K6;
                serializer.write<double>(distortionRational6k[6]); // P1;
                serializer.write<double>(distortionRational6k[7]); // P2;
            }
            else
            {
                for (int idx = 0; idx < 8; idx++)
                {
                    serializer.write<double>(0.0);
                }
            }

            serializer.write<double>(data.Calibration.GetFocalBounds().Lower);
            serializer.write<double>(data.Calibration.GetFocalBounds().Upper);
            serializer.write<double>(data.Calibration.GetCalibrationSize().Width);
            serializer.write<double>(data.Calibration.GetCalibrationSize().Height);
            serializer.write(data.FormatOfPixels);
        }

        void SerializeCameraSettings(fstream& outputStream, const mira::CameraSettings& cameraSettings)
        {
            binary_magestream<std::iostream> serializer{ outputStream };

            serializer.write<uint32_t>(cameraSettings.GetPopulatedFields().data);
            serializer.write<uint32_t>(cameraSettings.GetWhiteBalance().value_or(0));
            serializer.write<hundred_nanoseconds>(cameraSettings.GetExposureTime().value_or(mira::HundredsNanoseconds{}));
            serializer.write<uint32_t>(cameraSettings.GetLensPosition().value_or(0));
            serializer.write<uint32_t>(cameraSettings.GetIsoSpeed().value_or(0));
            serializer.write<float>(cameraSettings.GetIsoExternalGain().value_or(0.0f));
            serializer.write<float>(cameraSettings.GetIsoDigitalGain().value_or(0.0f));
        }

        void SerializeFrameData(fstream& outputFile, const FileFrameData& data)
        {
            binary_magestream<std::iostream> serializer{ outputFile };

            serializer.write(data.CorrelationId);
            serializer.write(gsl::narrow<int64_t>(data.TimeStamp.count()));
            SerializeCameraSettings(outputFile, data.CameraSettings);
            serializer.write(data.Pixels.data(), data.Pixels.size());
        }
    }
}
