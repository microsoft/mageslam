// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "TimeUnits.h"
#include <vector>
#include <istream>
#include <cereal/cereal.hpp>
#include <arcana/macros.h>

namespace mage
{
    class SensorSample
    {
    public:
        
        enum class SampleType
        {
            Unitialized,
            Accelerometer,
            Gyrometer,
            Magnetometer,
            ImageFence //should be last so that all sensor samples with equal timestamps sort to before images
        };

        using Timestamp = std::chrono::time_point<std::chrono::system_clock, hundred_nanoseconds>;

        SensorSample() = default;
        SensorSample(const SensorSample& other) = default;
        SensorSample(SensorSample&& other) = default;
        SensorSample& operator=(const SensorSample& other) = default;
        SensorSample& operator=(SensorSample&& other) = default;

        SensorSample(SampleType type, const Timestamp& timestamp, std::vector<float> samples);

        const Timestamp& GetTimestamp() const;
        SampleType GetType() const;

        bool IsSaturated() const;
        const std::vector<float>& GetData() const;
        std::vector<float>& GetData();
     
        template <class Archive>
        void serialize(Archive & ar, std::uint32_t const version)
        {
            UNUSED(version);
            assert(version == 1);

            ar(
                cereal::make_nvp("Timestamp",m_timestamp),
                cereal::make_nvp("Type", m_type),
                cereal::make_nvp("Data", m_data)
            );
        }

        struct Compare
        {
            // needs strict weak ordering
            bool operator()(const SensorSample& lhs, const SensorSample& rhs) const
            {
                if (lhs.GetTimestamp() == rhs.GetTimestamp())
                {
                    //map uses x !< y && y !< x to find match
                    if (lhs.GetType() == rhs.GetType())
                        return false;

                    // We want any imu sample with the same timestamp as an image fence to be processed before the image fence.
                    return lhs.GetType() < rhs.GetType();
                }

                return (lhs.GetTimestamp() < rhs.GetTimestamp());
            }
        };

    protected:
        Timestamp m_timestamp{ hundred_nanoseconds::zero() };
        SampleType m_type{ SensorSample::SampleType::Unitialized };
        std::vector<float> m_data;
    };
   
}
CEREAL_CLASS_VERSION(mage::SensorSample, 1);
