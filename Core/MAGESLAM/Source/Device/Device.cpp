// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Device.h"
#include "SupportedDevices.h"
#include <assert.h>
namespace mage
{
    namespace device
    {
        CameraDevice GetCameraDevice(const mira::CameraType& cameraType)
        {
            switch (cameraType)
            {
            case mira::CameraType::Lumia950:
                return GetCameraDeviceForLumia950();

            case mira::CameraType::SurfacePro3:
                return GetCameraDeviceForSurfacePro3();

            case mira::CameraType::SurfaceBook:
                return GetCameraDeviceForSurfaceBook();

            case mira::CameraType::Unknown: // Intentional fall-through for unknown devices.
            default:
                assert(false && "unsupported device type");
                return GetCameraDeviceForLumia950();
            }
        }

        IMUCharacterization GetIMUCharacterization(const mira::DeviceType& deviceType, const mira::CameraType& cameraType)
        {
            switch (deviceType)
            {
            case mira::DeviceType::Lumia950:
                return GetIMUCharacterizationForLumia950();

            case mira::DeviceType::SurfacePro3:
            case mira::DeviceType::SurfaceBook:
                return {};

            case mira::DeviceType::Middlebury:  // intentional fallthrough
                assert(false && "This data should be gathered from the dataset itself, not hard-coded here");
                return {};

            case mira::DeviceType::Unknown:
            default:
                assert(false && "unsupported device type");
                return GetIMUCharacterizationForLumia950();
            }
        }
    }
}
