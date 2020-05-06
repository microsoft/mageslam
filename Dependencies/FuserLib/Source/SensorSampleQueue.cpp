// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "pch.h"
#include "SensorSampleQueue.h"
#include "SensorFilter.h"
#include <algorithm>
#include <assert.h>

namespace mage
{
    long long SensorSampleQueue::s_statsTotalSamplesAdded = 0;
    long long SensorSampleQueue::s_statsLateSamples = 0;
    size_t SensorSampleQueue::s_statsDeepestQueue = 0;

    // assumes saturated signals are not added (filtered by hardware specific code)
    // a double add is not detectable
    bool SensorSampleQueue::AddSample(const SensorSample& sample)
    {
        std::lock_guard<std::shared_mutex> guard(m_sampleQueueMutex);
        return AddSampleInternal(sample);
    }

    bool SensorSampleQueue::AddFence(const SensorSample::Timestamp& timestamp)
    {
        std::lock_guard<std::shared_mutex> guard(m_sampleQueueMutex);

        SensorSampleId fenceId{ SensorSample::SampleType::ImageFence, timestamp };
        if (m_samples.end() != m_samples.find(fenceId))
            return false;

        return AddSampleInternal({ SensorSample::SampleType::ImageFence, timestamp, {} });
    }

    bool SensorSampleQueue::RemoveFence(const SensorSample::Timestamp& fenceTimestamp)
    {
        std::lock_guard<std::shared_mutex> guard(m_sampleQueueMutex);

        SensorSampleId fenceId{ SensorSample::SampleType::ImageFence, fenceTimestamp };

        auto fence = m_samples.find(fenceId);
        if (fence != m_samples.end())
        {
            m_samples.erase(fence);
            return true;
        }

        assert(false && "Double remove of an image fence detected");
        return false;
    }
 
    size_t SensorSampleQueue::SampleCount() const
    {
        std::shared_lock<std::shared_mutex> guard(m_sampleQueueMutex);
        return m_samples.size();
    }

    bool SensorSampleQueue::SamplesAvailable() const
    {
        std::shared_lock<std::shared_mutex> guard(m_sampleQueueMutex);
        return !m_samples.empty();
    }
    
    bool SensorSampleQueue::AddSampleInternal(const SensorSample& sample)
    {
        if (sample.GetTimestamp() < m_lastPoppedTimestamp)
        {
            // don't apply samples that are behind the current state of the filter
            s_statsLateSamples++;
            return false;
        }

        s_statsTotalSamplesAdded++;

        m_samples.emplace(SensorSampleId{ sample.GetType(), sample.GetTimestamp() }, sample);

        s_statsDeepestQueue = std::max(s_statsDeepestQueue, m_samples.size());

        return true;
    }

    // collect all samples from the same time to apply into a single IMUFilterEvent 
    // don't proceed beyond an image fence event
    bool SensorSampleQueue::PopCorrelatedSamples(std::vector<SensorSample>& fevSamples)
    {
        constexpr SensorSample::Timestamp::duration limitToCorrelate = std::chrono::microseconds(100); // not a setting that will be tweaked

        fevSamples.clear();
        fevSamples.reserve(3);

        {
            std::lock_guard<std::shared_mutex> guard(m_sampleQueueMutex);

            // Add the next sample set that all have the same time stamp, stopping at an image fence.
            if (!m_samples.empty())
            {
                auto begin = m_samples.begin();
                auto firstTimestamp = begin->first.timestamp;
                auto end = std::find_if(begin, m_samples.end(), [firstTimestamp, limitToCorrelate](const auto& pair)
                {
                    assert(pair.first.timestamp >= firstTimestamp && "timestamp list not sorted");
                    return ( (pair.first.timestamp - firstTimestamp) > limitToCorrelate || pair.first.type == SensorSample::SampleType::ImageFence);
                });

                std::transform(begin, end, std::back_inserter(fevSamples), [](auto& pair) { return std::move(pair.second);  });

                if (!fevSamples.empty())
                {
                    // Update last processed timestamp.
                    m_lastPoppedTimestamp = fevSamples.back().GetTimestamp();

                    // Erase all the items we added.
                    m_samples.erase(begin, end);
                }
            }
        }

        return !fevSamples.empty();
    }
}
