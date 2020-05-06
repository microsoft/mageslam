// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once
#include <array>
#include <chrono>

#include "Utils/Constants.h"
#include "Data/TimeUnits.h"

namespace mage
{
    namespace device
    {
        struct IMUCharacterization
        {
            using Timestamp = std::chrono::time_point<std::chrono::system_clock, hundred_nanoseconds>;

            bool useMagnetometer = false;
            
            bool ApplySensitivityEstimation = false;        //estimate scale on gyro error. off without visual update
            float DefaultInitialBiasVarianceFactor = 1.0f;  //scales the intial variances for all three sensors
            float AccelSampleRateMS = 4;                    //expected sample rate for accelerometer in milliseconds
            float GyroSampleRateMS = 4;                     //expected sample rate for gyrometer in milliseconds
            float MagSampleRateMS = 16;                     //expected sample rate for magnetometer in milliseconds
            float AccelNoiseSigma = 0.007f;                 //measurement noise as standard deviation m/s^2
            float GyroNoiseSigma = 0.00095f;                //measurement noise as standard deviation rad/s
            float MagNoiseSigma = 1.3f;                     //measurement noise as standard deviation uT
            float AccelBiasSigma = g_GravityMetersPerSecPerSec * 1e-3f;         // 1 millig in m/s/s
            float GyroBiasSigma = 1e-3f;                    // 1 mrad/s
            float MagBiasSigma = 30.0f;                        // microteslas

            std::array<float, 4 * 4> BodyIMUToBodyCamera;   //describes the physical orientation of the sensors (column vector convention right handed)
            std::array<float, 4 * 4> BodyCameraToBodyIMU;   //just to save runtime perf
        };
    }

}
