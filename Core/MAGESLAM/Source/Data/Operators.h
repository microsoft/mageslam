// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Keyframe.h"
#include "Tracking\KeyframeBuilder.h"
#include "Map\MapPoint.h"

#include <memory>

namespace mage
{
    namespace operators
    {
        namespace keyframe
        {
            struct less
            {
                bool operator ()(const Keyframe* left, const Keyframe* right) const
                {
                    return left->GetId() < right->GetId();
                }

                bool operator ()(const std::shared_ptr<KeyframeBuilder>& left, const std::shared_ptr<KeyframeBuilder>& right) const
                {
                    return left->GetId() < right->GetId();
                }

                bool operator ()(const std::shared_ptr<KeyframeBuilder>& left, const Id<Keyframe>& right) const
                {
                    return left->GetId() < right;
                }

                bool operator ()(const Id<Keyframe>& left, const std::shared_ptr<KeyframeBuilder>& right) const
                {
                    return left < right->GetId();
                }

                bool operator ()(const proxy::Id<Keyframe>& left, const proxy::Id<Keyframe>& right) const
                {
                    return left.GetId() < right.GetId();
                }

                bool operator ()(const Id<Keyframe>& left, const Id<Keyframe>& right) const
                {
                    return left < right;
                }
            };

            struct equal_to
            {
                bool operator ()(const Keyframe* left, const Keyframe* right) const
                {
                    return left->GetId() == right->GetId();
                }

                bool operator ()(const std::shared_ptr<KeyframeBuilder>& left, const std::shared_ptr<KeyframeBuilder>& right) const
                {
                    return left->GetId() == right->GetId();
                }

                bool operator ()(const std::shared_ptr<KeyframeBuilder>& left, const Id<Keyframe>& right) const
                {
                    return left->GetId() == right;
                }

                bool operator ()(const Id<Keyframe>& left, const std::shared_ptr<KeyframeBuilder>& right) const
                {
                    return left == right->GetId();
                }
                
                bool operator ()(const Id<Keyframe>& left, const Id<Keyframe>& right) const
                {
                    return left == right;
                }
            };
        }
    }
}

namespace std
{
    template<>
    struct hash<mage::proxy::Id<mage::MapPoint>> : private hash<mage::Id<mage::MapPoint>>
    {
        size_t operator()(const mage::proxy::Id<mage::MapPoint>& mp) const
        {
            return hash<mage::Id<mage::MapPoint>>::operator()(mp.GetId());
        }
    };
}
