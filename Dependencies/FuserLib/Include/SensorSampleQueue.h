// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "SensorSample.h"
#include <experimental/filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace mage
{
    template <class F>
    class SensorFilter;

    class SensorSampleQueue
    {
    public:

        bool AddSample(const SensorSample& sample);

        bool AddFence(const SensorSample::Timestamp& timestamp);
        bool RemoveFence(const SensorSample::Timestamp& timestamp);
      
        bool PopCorrelatedSamples(std::vector<SensorSample>& fevSamples);

        size_t SampleCount() const;
        bool SamplesAvailable() const;
     
    private:
        struct SensorSampleId
        {
            SensorSample::SampleType type;
            SensorSample::Timestamp timestamp;
        };

        struct TimestampCompare
        {
            // needs strict weak ordering
            bool operator()(const SensorSampleId& lhs, const SensorSampleId& rhs) const
            {
                if (lhs.timestamp == rhs.timestamp)
                {
                    //map uses x !< y && y !< x to find match
                    if (lhs.type == rhs.type)
                        return false;

                    // We want any imu sample with the same timestamp as an image fence to be processed before the image fence.                  
                    return lhs.type < rhs.type;
                }

                return (lhs.timestamp < rhs.timestamp);
            }
        };
        bool AddSampleInternal(const SensorSample& sample);

        std::map<SensorSampleId, SensorSample, TimestampCompare> m_samples;
        mutable std::shared_mutex m_sampleQueueMutex;

        mutable std::mutex m_loggingQueueMutex;

        SensorSample::Timestamp m_lastPoppedTimestamp{ hundred_nanoseconds::zero() };

        static long long s_statsTotalSamplesAdded;
        static long long s_statsLateSamples;
        static size_t s_statsDeepestQueue;
    };
}