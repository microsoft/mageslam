// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <gsl\gsl>

#include "Data\Data.h"
#include "Data\Pose.h"
#include "opencv\cv.h"
#include "MageSettings.h"

namespace mage
{
    struct VOIKeyframe
    {
        const VolumeOfInterestSettings& VoiSettings;
        cv::Vec3f WorldPosition;
        cv::Vec3f Forward;
        cv::Vec3f Right;
        cv::Vec3f Up;
        float NearDepth;
        float FarDepth;

        // Cached values for performance.
        cv::Vec3f Centroid;
        float DistanceAlphaToXi;
        float ModifiedDistanceAlphaToOmega;

        VOIKeyframe(const Pose& pose, const Depth& depth, const VolumeOfInterestSettings& voiSettings);
        float TeardropScore(const cv::Vec3f&) const;
    };

    bool CalculateVolumeOfInterest(gsl::span<const VOIKeyframe> keyframes, const VolumeOfInterestSettings& voiSettings, AxisAlignedVolume& voi);
}