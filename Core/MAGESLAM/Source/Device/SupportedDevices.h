// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Device.h"

namespace mage
{
    namespace device
    {
        // supported Cameras
        CameraDevice GetCameraDeviceForLumia950();
        CameraDevice GetCameraDeviceForSurfacePro3();
        CameraDevice GetCameraDeviceForSurfaceBook();

        mage::Matrix GetExtrinsics(mira::CameraType cameraType);

        // supported IMUs
        IMUCharacterization GetIMUCharacterizationForLumia950();

        // typical mappings for each device
        std::map<mira::CameraType, const mage::CameraIdentity> GetDeviceCameraBindings(const mira::DeviceType& deviceType, const mira::RuntimeType& runtime, const StereoSettings& stereoSettings);
    }
}
