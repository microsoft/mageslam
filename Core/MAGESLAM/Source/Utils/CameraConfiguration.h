// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MageSlam.h"

namespace mage
{
    inline const MAGESlam::CameraConfiguration& FindCameraConfiguration(const CameraIdentity toFind, gsl::span<const MAGESlam::CameraConfiguration> configurations)
    {
        auto config = std::find_if(configurations.begin(), configurations.end(), [toFind](const MAGESlam::CameraConfiguration& cameraConfiguration)
        {
            return cameraConfiguration.CameraIdentity == toFind;
        });

        assert(config != configurations.end() && "unknown camera configuration");
        return *config;
    }
}
