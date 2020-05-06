// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Image/AnalyzedImage.h"
#include "Tracking/KeyframeBuilder.h"
#include "Tracking/PoseEstimator.h"

#include <arcana/expected.h>
#include <arcana/messaging/mediator.h>

#include <memory>

namespace mage
{
    struct AnalysisCompleted
    {
        std::shared_ptr<AnalyzedImage> Analyzed;
    };

    struct InitCompleted
    {};

    struct TrackingLost
    {};

    struct PoseEstimated
    {
        std::shared_ptr<KeyframeBuilder> Frame;
        bool UsingRelocalization{ false };

        // The basis frame ID and pose refer to the frame which is the "basis"
        // for the relative pose estimation.  In other words, the position
        // estimated for Frame is valid in a world where the frame BasisFrameId
        // is located at BasisPose.
        FrameId BasisFrameId{};
        Pose BasisPose{};

        PoseEstimated(
            const std::shared_ptr<KeyframeBuilder>& frame,
            bool usingRelocalization,
            FrameId basisFrameId,
            const Pose& basisPose
        ) : Frame{ frame },
            UsingRelocalization{ usingRelocalization },
            BasisFrameId{ basisFrameId },
            BasisPose{ basisPose }
        {}
    };

    struct HistoryUpdated
    {
        TrackingFrameHistory History;
    };

    struct PoseRefined
    {
        bool IsNewKeyframe{ false };
        std::shared_ptr<KeyframeBuilder> Frame;
        mira::unique_vector<Id<MapPoint>> MapPointsThatFailedScoring;
    };

    using TrackingMediator = mira::mediator<
        // Max size 72 is required for x64 builds
        mira::dispatcher<72>,
        AnalysisCompleted,
        InitCompleted,
        TrackingLost,
        std::shared_ptr<const PoseEstimated>,
        PoseRefined,
        HistoryUpdated>;
}
