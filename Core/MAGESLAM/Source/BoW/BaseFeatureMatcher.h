// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Types.h"
#include "Image\ORBDescriptor.h"
#include <vector>

namespace mage
{
    class Keyframe;

    class BaseFeatureMatcher
    {
    public:
        BaseFeatureMatcher() {}

        virtual ~BaseFeatureMatcher() {}

        virtual size_t QueryFeatures(const ORBDescriptor& descriptor, std::vector<ptrdiff_t>& matches) const = 0;

        virtual const Id<Keyframe>& GetId() const = 0;
    };
}