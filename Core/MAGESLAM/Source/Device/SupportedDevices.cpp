// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Device.h"
#include "Utils/cv.h"
#include "Utils/Constants.h"
#include "Utils/MageConversions.h"
#include "arcana/math.h"
#include <Plat/Device/device.h>
#include <math.h>

namespace mage
{
    namespace device
    {
        CameraDevice GetCameraDeviceForSurfacePro3()
        {
            CameraDevice cameraDevice{};

            float K1 = 0.f;
            float K2 = 0.f;
            float K3 = 0.f;
            float P1 = 0.f;
            float P2 = 0.f;

            cameraDevice.CameraType = mira::CameraType::SurfacePro3;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                // Calibrated off of Brent's SP3
                {
                    {0, 1845.75f / 1920.0f},        // fx  M, B
                    {0, 1840.4f / 1080.0f},         // fy  M, B
                    979.76f / 1920.0f,              // cx
                    573.47f / 1080.0f,              // cy
                    {0,0},                          // focal bounds (N/A)
                    {1920, 1080},                   // calibration size
                    { K1, K2, K3, P1, P2 }          // poly3k
                });

            return cameraDevice;
        }

        CameraDevice GetCameraDeviceForSurfaceBook()
        {
            CameraDevice cameraDevice{};

            float K1 = 0.f;
            float K2 = 0.f;
            float K3 = 0.f;
            float P1 = 0.f;
            float P2 = 0.f;

            cameraDevice.CameraType = mira::CameraType::SurfaceBook;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                // Calibrated off of Nick's Surface Book
                // note that surface books have a decent autofocus, so this model could be refined
                // current values are focussed at around 3/4 of a meter
                {
                    { 0, 1587.29f / 1920.0f },          // fx  M, B
                    { 0, 1585.59f / 1080.0f },          // fy  M, B
                    963.24f / 1920.0f,                  // cx
                    560.54f / 1080.0f,                  // cy
                    { 0,0 },                            // focal bounds (N/A)
                    { 1920, 1080 },                     // calibration size
                    { K1, K2, K3, P1, P2 }              // poly3k
                });

            return cameraDevice;
        }

        CameraDevice GetCameraDeviceForLumia950()
        {
            float K0 = 0.094227405f;
            float K1 = -0.350755726f;
            float K2 = 0.416357188f;
            float P0 = 0.0f;
            float P1 = 0.0f;

            CameraDevice cameraDevice{};

            cameraDevice.CameraType = mira::CameraType::Lumia950;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                { -0.0001100515625f, 0.81877777291667f },   //fx M, B
                { -0.0001882685185f, 1.45169039537037f },   //fy M, B
                0.506385416667f,                            //cx
                0.51153703703704f,                          //cy
                { 550.0f, 700.0f },                         //focal bounds
                { 1920, 1080 },                              //calibration size
                { K0, K1, K2, P0, P1 });                    //poly3k

            cameraDevice.DefaultCameraFocus = 650;

            return cameraDevice;
        }

        IMUCharacterization GetIMUCharacterizationForLumia950()
        {
            // The image from the sensor on the 950 is always based on landscape, so holding the phone in portrait mode will pin the mage camera space convention(x right, y down, z forward) 
            // so that x points down and y points left, z is still forward.
            // The imu is installed on the 950 and exposed by the sensors api so that with the phone in portrait mode it is y up to the top of phone, x to right, and z back towards the user.
            // To rotate from body camera to body imu  rotate around camera z - 90. then rotate around that x axis 180 degrees.
            // column major, math flows right to left
            //cv::Matx44f rotNeg90Z = mage::RotationZForCoordinateFrames(mira::deg2rad(-mira::QUARTER_CIRCLE_DEGREES<float>));
            //cv::Matx44f rot180X = mage::RotationXForCoordinateFrames(mira::deg2rad(mira::HALF_CIRCLE_DEGREES<float>));
            //cv::Matx44f bodyIMUToBodyCameraRotation = rot180X * rotNeg90Z;

            // Ran the calibration tool with data using their 3d calibration marker board thanks to Eric Huang.
            // These values are taken straight from the calibration.json file "Rt" property.
            cv::Matx44f bodyCameraToBodyIMU{
                -0.0023918196093291044, -0.99980247020721436, 0.019730480387806892, 0.02890799380838871,
                -0.99998271465301514, 0.0024972527753561735, 0.0053207604214549065, 0.10563744604587555,
                -0.0053689810447394848, -0.019717413932085037, -0.99979120492935181, 0.0064810086041688919,
                0, 0, 0, 1
            };

            bool inverted = false;
            cv::Matx44f bodyIMUToBodyCamera = bodyCameraToBodyIMU.inv(cv::DECOMP_LU, &inverted);
            assert(inverted && "Failed to invert matrix");

            IMUCharacterization imuCharacterization{};
            imuCharacterization.useMagnetometer = false;                    //results on 950 are worse with mag, than without
            imuCharacterization.ApplySensitivityEstimation = false;         //can turn on when we have visual update
            imuCharacterization.DefaultInitialBiasVarianceFactor = 1.0f;    //variances are fine
            imuCharacterization.AccelSampleRateMS = 4.0f;                   //this is the requested value, measured varies based on phone state
            imuCharacterization.GyroSampleRateMS = 4.0f;
            imuCharacterization.MagSampleRateMS = 16.0;
            imuCharacterization.AccelNoiseSigma = 250.0f * 1e-6f * g_GravityMetersPerSecPerSec * sqrtf(0.5f * 1.0f / (1e-3f * imuCharacterization.AccelSampleRateMS));       // micro g's/sqrtHz to m/s^2
            imuCharacterization.GyroNoiseSigma = mira::deg2rad(20.0f * 1e-3f) * sqrtf(0.5f  * 1.0f / (1e-3f * imuCharacterization.GyroSampleRateMS));  // millidegrees/sec/sqrtHz to rad/s, (assume bandwidth is about half the sample rate). 
            imuCharacterization.MagNoiseSigma = 0.7f;                                                                                                  // uT
            imuCharacterization.AccelBiasSigma = 80.0f * g_GravityMetersPerSecPerSec * 1e-3f;            // 80 millig in m/s/s
            imuCharacterization.GyroBiasSigma = mira::deg2rad(20.0f) * 1e-3f;    // 20 degrees/sec = (20 * 3.14) / 180 mrad/s
            imuCharacterization.MagBiasSigma = 30.0f;                           //microtesla

            imuCharacterization.BodyIMUToBodyCamera = ArrayFromMat(bodyIMUToBodyCamera);
            imuCharacterization.BodyCameraToBodyIMU = ArrayFromMat(bodyCameraToBodyIMU);

            return imuCharacterization;
        }

        //returns a passive matrix that takes you from the device's origin (panel origin) in CAD space (device in portrait, looking at screen, x right, y up, z to user)
        // to the camera space center of the sensor in opencv convention(camera center, looking through camera x right, y down, z forward)
        mage::Matrix GetExtrinsics(mira::CameraType cameraType)
        {
            // sensors appear to be oriented with image upper left in the top right corner when device is held in portrait
            cv::Matx44f cadToSensorRotation{ 0, -1,  0, 0,
                                            -1,  0,  0, 0,
                                             0,  0, -1, 0,
                                             0,  0,  0, 1 };
            switch (cameraType)
            {
            case mira::CameraType::Lumia950:
            case mira::CameraType::SurfacePro3:
            case mira::CameraType::SurfaceBook:
            {
                return ToMageMat(cv::Matx44f::eye());
            }
            default:
                assert(false && "Intrinsics not provided for camera type");
                return ToMageMat(cv::Matx44f::zeros());
            }
        }

        std::map<mira::CameraType, const mage::CameraIdentity> GetDeviceCameraBindings(const mira::DeviceType& deviceType, const mira::RuntimeType& runtime, const StereoSettings& stereoSettings)
        {
            switch (deviceType)
            {
            case mira::DeviceType::Lumia950:
                return { {mira::CameraType::Lumia950,  mage::CameraIdentity::MONO} };
            case mira::DeviceType::SurfacePro3:
                return { { mira::CameraType::SurfacePro3, mage::CameraIdentity::MONO } };
            case mira::DeviceType::SurfaceBook:
                return { { mira::CameraType::SurfaceBook, mage::CameraIdentity::MONO } };
            default:
                throw std::invalid_argument("Unknown runtime type");
            }
        }
    }
}
