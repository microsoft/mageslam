// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <string>

namespace mira
{
    enum class DeviceType;

    // Keeping Device Properties for compability with BIN recordings for Lumia 950.
    struct DeviceProperties
    {
        std::string Id;
        std::string DeviceName;
        std::string DeviceManufacturer;
        std::string DeviceHardwareVersion;
        std::string DeviceFirmwareVersion;
    };

    DeviceProperties GetDeviceProperties();

    struct SystemProperties
    {
        std::string DeviceGuid;
        std::string DeviceUniqueId;
        std::string DeviceName;
        DeviceType DeviceType;
        std::string DeviceManufacturer;
        std::string DeviceHardwareVersion;
        std::string DeviceFirmwareVersion;
        std::string DeviceFamily;
        std::string DeviceFamilyFriendlyName;
        std::string OSBuildNumber;
        std::string OSBranchName;
        std::string AppName;
        std::string AppArchitecture;
        std::string AppBuildNumber;
    };

    SystemProperties GetSystemProperties(DeviceType deviceType);
}
