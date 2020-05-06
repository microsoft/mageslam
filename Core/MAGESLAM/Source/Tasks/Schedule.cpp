// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Schedule.h"

namespace mage
{
    mira::state_machine_state<bool> InitializeState{ "InitializeState" };
    mira::state_machine_state<bool> TrackingReadState{ "TrackingReadState" };

    mira::state_machine_state<void> KeyframeInsertionAndMapPointCullingState{ "KeyframeInsertionAndMapPointCulling" };
    mira::state_machine_state<void> MapPointCreationState{ "MapPointCreationState" };

    mira::state_machine_state<bool> BundleAdjustFirstWriteToMapState{ "BundleAdjustFirstWriteToMapState" };
    mira::state_machine_state<bool> BundleAdjustNthWriteToMapState{ "BundleAdjustNthWriteToMapState" };

    mira::state_machine_state<void> KeyframeCullingState{ "KeyframeCullingState" };

    mira::state_machine_state<bool> LoopDetectionState{ "LoopDetectionState" };
    mira::state_machine_state<LoopClosureTrackingUpdate> StartLoopClosureState{ "StartLoopClosureState" };
    mira::state_machine_state<void> EndLoopClosureState{ "EndLoopClosureState" };
}
