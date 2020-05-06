// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "SensorSample.h"

#include <Device/ImuCharacterization.h>
#include <array>
#include <vector>
#include <fstream>
#include <memory>
#include <shared_mutex>

namespace ST
{
    class VFTFilter;
    struct IMUConfig;
    class IMUFilterCAS;
    class SimpleIMUFilter;
}

namespace mage
{
    class ISensorFilter
    {
    public:
        virtual bool IsValid() const = 0;
        virtual bool HasGoodGravity() const = 0;        
        virtual void Reset(const SensorSample::Timestamp& resetTime) = 0;
        virtual void PredictTo(const SensorSample::Timestamp& timestamp) = 0;
        virtual const mage::device::IMUCharacterization& GetIMUSettings() const = 0;
        virtual SensorSample::Timestamp GetLastProcessedTimestamp() const = 0;
        virtual bool GetWorldGravity(std::array<float, 3>& gravityWorld) const = 0;
        virtual std::array<float, 3> GetBodyLinearVelocity(SensorSample::Timestamp& fromTime) const = 0;
        virtual std::array<float, 3> GetBodyAngularVelocity(SensorSample::Timestamp& fromTime) const = 0;
        virtual std::array<float, 3> GetBodyLinearAcceleration(SensorSample::Timestamp& fromTime) const = 0;
        virtual std::array<float, 3> GetBodyAngularAcceleration(SensorSample::Timestamp& fromTime) const = 0;
        virtual std::array<float, 4 * 4> GetDeltaPose(const SensorSample::Timestamp& fromTime) const = 0;
        virtual std::array<float, 4 * 4>  GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>& poseCovariance) = 0;
        virtual std::chrono::system_clock::time_point ProcessCorrelatedSamples(const std::vector<SensorSample>& sampleEvents) = 0;
    };

    template <typename F>
    class SensorFilter : public ISensorFilter
    {
    public:
        
        SensorFilter() = delete;
        SensorFilter(const mage::device::IMUCharacterization& imuCharacterization);
        virtual ~SensorFilter();

        std::array<float, 3> GetBodyLinearVelocity(SensorSample::Timestamp& fromTime) const;
        std::array<float, 3> GetBodyAngularVelocity(SensorSample::Timestamp& fromTime) const;
        std::array<float, 3> GetBodyLinearAcceleration(SensorSample::Timestamp& fromTime) const;
        std::array<float, 3> GetBodyAngularAcceleration(SensorSample::Timestamp& fromTime) const;
        std::array<float, 4 * 4> GetDeltaPose(const SensorSample::Timestamp& fromTime) const;

        bool GetWorldGravity(std::array<float, 3>& gravityWorld) const;
        bool HasGoodGravity() const;

        SensorSample::Timestamp GetLastProcessedTimestamp() const;
        std::chrono::system_clock::time_point ProcessCorrelatedSamples(const std::vector<SensorSample>& sampleEvents);

        void Reset(const SensorSample::Timestamp& resetTime);
        const  mage::device::IMUCharacterization& GetIMUSettings() const { return m_imuCharacterization; }
        
        bool IsValid() const;
        
        void PredictTo(const SensorSample::Timestamp& timestamp);

        std::unique_ptr<F> ExtractFilter();

    protected:

        // don't take the mutex
        SensorSample::Timestamp GetLastProcessedTimestampInternal() const;
        void ResetInternal(const SensorSample::Timestamp& resetTime);
        void PredictInternal(const SensorSample::Timestamp& timestamp);
        bool IsValidInternal() const;

        bool m_initialized = false;
        mutable std::shared_mutex m_mutex;
        std::unique_ptr<F> m_filter;
        std::unique_ptr<ST::IMUConfig> m_config;
        
    private:

        //doesn't take the mutex
        virtual void OnResetInternal(const SensorSample::Timestamp& /*resetTime*/) {}

        SensorSample::Timestamp m_resetTimestamp{ SensorSample::Timestamp::duration::zero() };
        SensorSample::Timestamp m_lastProcessedTimestamp{ SensorSample::Timestamp::duration::zero() };
        
        mage::device::IMUCharacterization m_imuCharacterization; 
       
    };

    class SensorFilter3Dof : public SensorFilter<ST::VFTFilter>
    {
    public:        
        SensorFilter3Dof(const mage::device::IMUCharacterization& settings) : SensorFilter<ST::VFTFilter>(settings) {} 
        virtual ~SensorFilter3Dof() = default;

        std::array<float, 4 * 4>  GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>& poseCovariance);

        void AddVisualRotationUpdate(const std::array<float, 3 * 3>& matWorldIMUToCameraFromMage, const SensorSample::Timestamp & imageTime, const std::array<double, 3 * 3> poseCovariance);
        void AddVisualRotationUpdate(const std::array<float, 3 * 3>& matWorldIMUToCameraFromMage, const SensorSample::Timestamp & imageTime, double visualStdDev);
    private: 

        static std::array<double, 6 * 6> GetCovarianceForInverse(const std::array<float, 3 * 3>& invertedMat, const std::array<double, 6 * 6>& covarianceUninverted);
    };
    
    class SensorFilter6Dof : public SensorFilter<ST::IMUFilterCAS>
    {
    public:
        SensorFilter6Dof(const mage::device::IMUCharacterization& imuCharacterization) : SensorFilter<ST::IMUFilterCAS>(imuCharacterization) {}
        SensorFilter6Dof(std::unique_ptr<SensorFilter3Dof> filter3Dof);
        virtual ~SensorFilter6Dof() {}

        std::array<float, 4 * 4>  GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>& poseCovariance); //rename
        std::array<float, 4 * 4> GetDeltaPose(const SensorSample::Timestamp& fromTime) const;

        SensorSample::Timestamp GetResetDeltaPoseTimestamp() const { return m_deltaPoseResetTime; }
        void ResetDeltaPoseIntegration(const SensorSample::Timestamp& resetTime);

        void ResetPoseTranslation();

        bool HasGoodScale() const;
        bool GetEstimatedVisualToMetersScale(float& visualScaleToMeters) const;

        bool AddVisualPoseDeltaUpdate(const std::array<float, 4 * 4>& matIMU0ToIMU1FromMage, const SensorSample::Timestamp& image1Time, const std::array<double, 6 * 6>& deltaPoseCovariance);
        bool AddVisualPoseDeltaUpdate(const std::array<float, 4 * 4>& matIMU0ToIMU1FromMage, const SensorSample::Timestamp& image1Time, double visualStdDev);

        void SwitchFilterOrigin(const std::array<float, 4 * 4>& matNewToOld, const std::array<double, 6 * 6>& poseCovarianceInNewOrigin);

        void SetPoseWorldPosition(float xWorldPos, float yWorldPos, float zWorldPos, const std::array<double, 3 * 3>& covTranslation);

        static std::array<double, 6 * 6> GetCovarianceForMatMul(const std::array<float, 4 * 4>& matA, const std::array<double, 6 * 6>& covA, const std::array<double, 6 * 6>& covB);

       private:

        static std::array<double, 6 * 6> GetCovarianceForInverse(const std::array<float, 4 * 4>& invertedMat, const std::array<double, 6 * 6>& covarianceUninverted);

        void OnResetInternal(const SensorSample::Timestamp& resetTime);
        bool HasGoodScaleInternal() const;

        SensorSample::Timestamp m_deltaPoseResetTime;

        const double GoodScaleCovToScaleRatio = 0.05;
    
    };

    class SensorFilterSimple6Dof : public SensorFilter<ST::SimpleIMUFilter>
    {
    public:
        SensorFilterSimple6Dof(const mage::device::IMUCharacterization& imuCharacterization) : SensorFilter<ST::SimpleIMUFilter>(imuCharacterization) {}

        std::array<float, 4 * 4>  GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>&);

        std::array<float, 4 * 4> GetDeltaPose(const SensorSample::Timestamp& fromTime) const;

        SensorSample::Timestamp GetResetDeltaPoseTimestamp() const { return m_deltaPoseResetTime; }
        void ResetDeltaPoseIntegration(const SensorSample::Timestamp& resetTime);

        void SimpleSwitchFilterOrigin(const std::array<float, 4 * 4>& curCameraToMageWorld, const SensorSample::Timestamp& resetTime);


    private:

        void OnResetInternal(const SensorSample::Timestamp& resetTime);

        SensorSample::Timestamp m_deltaPoseResetTime;


    };
    
}
