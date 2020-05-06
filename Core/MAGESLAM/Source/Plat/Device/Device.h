// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    enum class CameraType
    {
        Unknown,
        Lumia950,
        SurfacePro3,
        SurfaceBook,
    };

    inline constexpr char const* ToString(CameraType type)
    {        
        switch (type)
        {
        case CameraType::Unknown:
            return "Unknown";
        case CameraType::Lumia950:
            return "Lumia950";
        case CameraType::SurfaceBook:
            return "SurfaceBook";
        case CameraType::SurfacePro3:
            return "SurfacePro3";
        default:
            return "";
        }
    }

    enum class DeviceType
    {
        Unknown,
        Lumia950,
        SurfacePro3,
        SurfaceBook,
        Middlebury,
    };

    enum class RuntimeType
    {
        Mono,
        Stereo
    };
}
