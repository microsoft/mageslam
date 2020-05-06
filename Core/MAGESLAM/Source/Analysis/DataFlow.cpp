// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "DataFlow.h"

#include "Utils\Logging.h"
#include "Analysis\binary_iterators.h"

namespace mage
{
    DataFlow::~DataFlow()
    {
        LogStatistic(L"DataFlow", L"Input", m_inputBytes, 1);

        for (auto& fun : m_output)
        {
            fun();
        }

        LogStatistic(L"DataFlow", L"Output", m_outputBytes, 0);
    }

    void DataFlow::LogInput(const void*, size_t size)
    {
        m_inputBytes += size;
    }

    void DataFlow::LogOutput(const void*, size_t size)
    {
        m_outputBytes += size;
    }
}