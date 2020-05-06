// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/timer.h>

#include "Map/ThreadSafeMap.h"
#include "Map/ThreadSafePoseHistory.h"
#include "Tasks/MageContext.h"
#include "Fuser/Fuser.h"

namespace mage
{
    struct SkeletonData
    {
        ThreadSafeMap* map;
        ThreadSafePoseHistory* history;
        MageContext* sharedData;
        Fuser* fuser;
    };

    struct FossilizedSkeletonData
    {
        const PoseHistory* History;
        std::vector<Position> MapPoints;
    };
}
