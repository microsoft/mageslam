// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseFeatureMatcher.h"
#include "OnlineBow.h"

#include <gsl\gsl>
#include <unordered_map>

namespace mage
{
    class Keyframe;

    class OnlineBowFeatureMatcher : public BaseFeatureMatcher
    {
    public:
        OnlineBowFeatureMatcher(const OnlineBow& onlineBow, const Id<Keyframe>& id, gsl::span<const ORBDescriptor> features);

        virtual size_t QueryFeatures(const ORBDescriptor& descriptor, std::vector<ptrdiff_t>& matches) const override;

        virtual const Id<Keyframe>& GetId() const override;

    private:
        // a Map of NodeId <==> the indexes of the features which are assigned to the Node.
        std::unordered_map<ptrdiff_t, std::vector<ptrdiff_t>> m_featureMap;
        const OnlineBow& m_onlineBow;
        const Id<Keyframe> m_id;
    };
}