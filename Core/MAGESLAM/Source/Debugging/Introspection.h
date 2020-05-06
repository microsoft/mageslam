// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Introspector.h"

#include <vector>
#include <mutex>

namespace mage
{
    class Introspection: public Introspector
    {
    public:
        void AddIntrospector(Introspector& introspector)
        {
            std::lock_guard<std::mutex> guard{ m_mutex };
            m_introspectors.push_back(&introspector);
        }

        virtual void Introspect(const InitializationData& data) override
        {
            std::lock_guard<std::mutex> guard{ m_mutex };
            for (Introspector* ptr : m_introspectors)
            {
                ptr->Introspect(data);
            }
        }

        virtual void IntrospectEstimatedPose(const mage::FrameId& frameId, const mage::Matrix& viewMatrix) override
        {
            std::lock_guard<std::mutex> guard{ m_mutex };
            for (Introspector* ptr : m_introspectors)
            {
                ptr->IntrospectEstimatedPose(frameId,viewMatrix);
            }
        }

        virtual void IntrospectAnalyzedImage(const mage::FrameData& frame, const mage::AnalyzedImage& image) override
        {
            std::lock_guard<std::mutex> guard{ m_mutex };
            for (Introspector* ptr : m_introspectors)
            {
                ptr->IntrospectAnalyzedImage(frame, image);
            }
        }

    private:
        std::mutex m_mutex;
        std::vector<Introspector*> m_introspectors;
    };
}
