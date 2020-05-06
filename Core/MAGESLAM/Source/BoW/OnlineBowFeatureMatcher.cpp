// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "OnlineBowFeatureMatcher.h"

namespace mage
{
    OnlineBowFeatureMatcher::OnlineBowFeatureMatcher(const OnlineBow& onlineBow, const Id<Keyframe>& id, gsl::span<const ORBDescriptor> features)
        : m_onlineBow{ onlineBow },
          m_id{ id }
    {
        for (ptrdiff_t i = 0; i < features.size(); ++i)
        {
            auto leafId = m_onlineBow.FindLeafNode(features[i]);
            auto insertion = m_featureMap.insert({ leafId,{} });
            insertion.first->second.emplace_back(i);
        }
    }

    size_t OnlineBowFeatureMatcher::QueryFeatures(const ORBDescriptor& descriptor, std::vector<ptrdiff_t>& matches) const
    {
        matches.clear();

        auto leafId = m_onlineBow.FindLeafNode(descriptor);
        const auto result = m_featureMap.find(leafId);
        if (result != m_featureMap.end())
        {
            matches = result->second;
        }
        return matches.size();
    }

    const Id<Keyframe>& OnlineBowFeatureMatcher::GetId() const
    {
        return m_id;
    }
}
