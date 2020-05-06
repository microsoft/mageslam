// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data/Data.h"
#include "MageSettings.h"
#include "IMUCharacterization.h"
#include <stdint.h>
#include <Plat/Device/Device.h>

namespace mage
{
    namespace device
    {
        struct CameraDevice
        {
            mira::CameraType CameraType = mira::CameraType::Unknown;
            mage::calibration::LinearFocalLengthModel Model;
            uint64_t DefaultCameraFocus;
        };

        CameraDevice GetCameraDevice(const  mira::CameraType& cameraType);
        IMUCharacterization GetIMUCharacterization(const  mira::DeviceType& deviceType, const mira::CameraType& cameraType);
    }
}
