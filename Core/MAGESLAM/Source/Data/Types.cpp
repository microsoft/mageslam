// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Types.h"

namespace mage
{
    class Keyframe;
    class MapPoint;

    std::atomic<IdT> id_generator<IdT, Keyframe>::s_seed{ 0 };
    std::atomic<IdT> id_generator<IdT, MapPoint>::s_seed{ 0 };
}