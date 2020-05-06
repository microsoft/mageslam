// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "pch.h"
#include "SensorSample.h"
#include <fstream>
#include <ostream>
#include <assert.h>
#include <chrono>
#include <cereal\cereal.hpp>

namespace mage
{

    SensorSample::SensorSample(SampleType type, const Timestamp& timestamp, std::vector<float> samples)
        : m_timestamp(timestamp)
        , m_type(type)
    {
        m_data = std::move(samples);
    }

    const SensorSample::Timestamp& SensorSample::GetTimestamp() const
    {
        return m_timestamp;
    }

    SensorSample::SampleType SensorSample::GetType() const
    {
        return m_type;
    }

    bool SensorSample::IsSaturated() const
    {
        return false;
    }

    const std::vector<float>& SensorSample::GetData() const
    {
        return m_data;
    }

    std::vector<float>& SensorSample::GetData()
    {
        return m_data;
    }
}
