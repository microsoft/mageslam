// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "pch.h"
#include "LegacySerialization.h"
#include <assert.h>

namespace mage
{    
    enum class LegacySampleType  // sensor sample enum ordering pre mage binary version 3
    {
        Unitialized,
        ImageFence,
        Accelerometer,
        Gyrometer,
        Magnetometer
    };
        
    SensorSample::SampleType ConvertLegacySample(const LegacySampleType& legacySampleType)
    {
        switch (legacySampleType)
        {
        case LegacySampleType::Unitialized:
            return SensorSample::SampleType::Unitialized;
        case LegacySampleType::ImageFence:
            return SensorSample::SampleType::ImageFence;
        case LegacySampleType::Accelerometer:
            return SensorSample::SampleType::Accelerometer;
        case LegacySampleType::Gyrometer:
            return SensorSample::SampleType::Gyrometer;
        case LegacySampleType::Magnetometer:
            return SensorSample::SampleType::Magnetometer;
        default:
            assert(false && "Unexpected type");
            return SensorSample::SampleType::Unitialized;
        }
    }

    unsigned int GetLegacyExpectedNumDataElements(const SensorSample::SampleType& sampleType)
    {
        switch (sampleType)
        {
        case SensorSample::SampleType::Unitialized:
            assert(false);
            return 0;
        case SensorSample::SampleType::ImageFence:
            return 0;
        case SensorSample::SampleType::Accelerometer:
            return 3;
        case SensorSample::SampleType::Gyrometer:
            return 3;
        case SensorSample::SampleType::Magnetometer:
            return 4;
        default:
            assert(false);
            return 0;
        }
    }

    bool LegacyDebugRead(std::istream& inputFileStream, const uint64_t& binVersion, SensorSample& outSample)
    {
        if (!inputFileStream.good())
            return false;

        if (binVersion >= 3)
            return false;

        char delim;
        int type;

        SensorSample::SampleType sampleType;
        SensorSample::Timestamp  sampleTimestamp;
        std::vector<float> sampleData;

        switch (binVersion)
        {
        case 0:
        {
            unsigned long long timestamp;
            if (!(inputFileStream >> timestamp))
                return false;
            sampleTimestamp = SensorSample::Timestamp{ hundred_nanoseconds(timestamp / 100) };
        }
        break;
        case 1:
        case 2:
        {
            long long timestamp;
            if (!(inputFileStream >> timestamp))
                return false;
            sampleTimestamp = SensorSample::Timestamp{ hundred_nanoseconds(timestamp) };
            break;
        }
        default:
            assert(false && "unexpected binary file version in imu.csv loading");
        }

        if (!(inputFileStream >> delim))
            return false;

        if (!(inputFileStream >> type))
            return false;

        sampleType = ConvertLegacySample(static_cast<LegacySampleType>(type));

        if (!(inputFileStream >> delim))
            return false;

        unsigned int numDataElements = GetLegacyExpectedNumDataElements(sampleType);
        sampleData.clear();
        sampleData.reserve(numDataElements);

        for (unsigned int idx = 0; idx < numDataElements; idx++)
        {
            float data;

            if (!(inputFileStream >> data))
                return false;
            if (!(inputFileStream >> delim))
                return false;

            if (sampleType == SensorSample::SampleType::Accelerometer)
            {
                // previously all accelerometer data was negated in scan app when received from sensor stack
                // so this negation brings it back to the the same sign the windows api gives us.
                data *= -1.0f;
            }
            sampleData.push_back(data);
        }

        outSample = SensorSample{ sampleType, sampleTimestamp, std::move(sampleData) };
        return true;
    }

}
