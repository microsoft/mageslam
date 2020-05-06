// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/scheduling/state_machine.h>

namespace mage
{
    struct LoopClosureTrackingUpdate;

    extern mira::state_machine_state<bool> InitializeState;
    extern mira::state_machine_state<bool> TrackingReadState;

    extern mira::state_machine_state<void> KeyframeInsertionAndMapPointCullingState;
    extern mira::state_machine_state<void> MapPointCreationState;

    extern mira::state_machine_state<bool> BundleAdjustFirstWriteToMapState;
    extern mira::state_machine_state<bool> BundleAdjustNthWriteToMapState;

    extern mira::state_machine_state<void> KeyframeCullingState;

    extern mira::state_machine_state<bool> LoopDetectionState;
    extern mira::state_machine_state<LoopClosureTrackingUpdate> StartLoopClosureState;
    extern mira::state_machine_state<void> EndLoopClosureState;
}
