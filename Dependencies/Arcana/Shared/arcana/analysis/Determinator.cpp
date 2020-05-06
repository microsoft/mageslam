// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "determinator.h"
#include <assert.h>  // for assert

namespace mira
{
    std::mutex determinator::m_detlock;
    std::map<std::string, determinator> determinator::m_determinators;

    void determinator::process_crc(size_t value)
    {
#ifndef NDEBUG
        size_t index = m_samples.size();
        if (index < m_truth.size())
        {
            // validate relative to truth
            if (m_truth[index] != value)
            {
                assert(false && "Determinism Check Failed");
            }
        }
#endif

        m_samples.push_back(value);
    }
}
