// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "UnfilteredIMU.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils/cv.h"
#include <boost/optional.hpp>

#include <arcana/containers/sorted_vector.h>
#include <arcana/analysis/object_trace.h>
#include <arcana/analysis/data_point.h>

namespace E = Eigen;

namespace mage
{
    using seconds = std::chrono::duration<double>;

    struct alignas(16) UnfilteredIMU::Impl
    {
        E::Matrix4d m_bodyIMUToBodyCamera = E::Matrix4d::Identity();
        E::Matrix3d m_bodyIMUToBodyCameraRotationOnly = E::Matrix3d::Identity();
        E::Matrix4d m_bodyCameraToBodyIMU = E::Matrix4d::Identity();

        mage::SensorSample::Timestamp m_gyroTime{};
        mage::SensorSample::Timestamp m_accelTime{};

        mira::sorted_vector<mage::SensorSample, mage::SensorSample::Compare> m_samples;

        mage::SensorSample::Timestamp m_previousTs{};
        mage::Pose m_previousPose;

        Eigen::Vector3d Gravity = Eigen::Vector3d::Zero();

        Eigen::Vector3d Acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d Velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d Position = Eigen::Vector3d::Zero();

        E::AngleAxisd AngularVelocity = E::AngleAxisd::Identity();
        Eigen::Quaterniond Orientation = Eigen::Quaterniond::Identity();

        void ProcessGyroUntil(mage::SensorSample::Timestamp ts)
        {
            if (m_gyroTime == mage::SensorSample::Timestamp{})
                m_gyroTime = ts;

            seconds dt = ts - m_gyroTime;

            E::AngleAxisd av = AngularVelocity;
            av.angle() *= dt.count();

            Orientation = Orientation * av;
            Orientation.normalize();

            m_gyroTime = ts;
        }

        void SetAngularVelocity(Eigen::Map<const Eigen::Vector3f> value)
        {
            Eigen::Vector3f angularVelocityInCameraSpace{
                value[1],
                value[0],
                -value[2]
            };
            AngularVelocity = E::AngleAxisd(mira::deg2rad<double>(angularVelocityInCameraSpace.x()), E::Vector3d::UnitX())
                * E::AngleAxisd(mira::deg2rad<double>(angularVelocityInCameraSpace.y()), E::Vector3d::UnitY())
                * E::AngleAxisd(mira::deg2rad<double>(angularVelocityInCameraSpace.z()), E::Vector3d::UnitZ());
        }

        void ProcessAccelerationUntil(mage::SensorSample::Timestamp ts)
        {
            if (m_accelTime == mage::SensorSample::Timestamp{})
                m_accelTime = ts;

            seconds dt = ts - m_accelTime;

            E::Vector3d vi = Velocity;

            Velocity += dt.count() * (Acceleration - Gravity);
            Position += 0.5 * (Velocity + vi) * dt.count();

            FIRE_OBJECT_TRACE("IMU Linear Velocity.UnfilteredIMU", this, (mira::make_data_point<float>(
                ts,
                (float)Velocity.norm()
            )));

            m_accelTime = ts;
        }

        void SetLinearAcceleration(Eigen::Map<const Eigen::Vector3f> value)
        {
            Eigen::Vector3f sensorAccelInCameraSpace{
                value[1],
                value[0],
                -value[2]
            };

            Acceleration = Orientation * sensorAccelInCameraSpace.cast<double>();

            Gravity = 0.9 * Gravity + 0.1 * Acceleration;
        }

        void ProcessPose(mage::SensorSample::Timestamp ts, const mage::Pose& pose)
        {
            assert(ts >= m_previousTs);

            PredictUpTo(ts);

            auto transform = Decompose(pose.GetInverseViewMatrix());

            Orientation = transform.second.cast<double>();
            Position = ToMap(transform.first).cast<double>();

            if (m_previousTs == mage::SensorSample::Timestamp{})
            {
                Velocity = E::Vector3d::Zero();
                AngularVelocity = E::AngleAxisd::Identity();
            }
            else
            {
                seconds dt = ts - m_previousTs;
                auto previous = Decompose(m_previousPose.GetInverseViewMatrix());
            
                Velocity = (Position - ToMap(previous.first).cast<double>()) / dt.count();
            
                AngularVelocity = (transform.second * previous.second.inverse()).cast<double>();
                AngularVelocity.angle() /= dt.count();
            }

            m_previousPose = pose;
            m_previousTs = ts;
        }

        void PredictUpTo(mage::SensorSample::Timestamp timestamp)
        {
            auto from = m_samples.begin();
            auto to = std::find_if(from, m_samples.end(), [timestamp](const mage::SensorSample& sample)
            {
                return sample.GetTimestamp() > timestamp;
            });

            for (auto itr = from; itr != to; itr++)
            {
                if (itr->GetType() == mage::SensorSample::SampleType::Gyrometer)
                {
                    ProcessGyroUntil(itr->GetTimestamp());
                    SetAngularVelocity({ itr->GetData().data(), 3 });
                }
                else if (itr->GetType() == mage::SensorSample::SampleType::Accelerometer)
                {
                    ProcessAccelerationUntil(itr->GetTimestamp());
                    SetLinearAcceleration({ itr->GetData().data(), 3 });
                }
            }

            m_samples.erase(from, to);

            ProcessGyroUntil(timestamp);
            ProcessAccelerationUntil(timestamp);
        }
    };

    UnfilteredIMU::UnfilteredIMU(const device::IMUCharacterization& imuCharacterization)
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}) }
    {
        m_impl->m_bodyIMUToBodyCamera = E::Map<const E::Matrix4f>(imuCharacterization.BodyIMUToBodyCamera.data(), 4, 4).cast<double>();
        m_impl->m_bodyIMUToBodyCameraRotationOnly = m_impl->m_bodyIMUToBodyCamera.block<3, 3>(0, 0);
        m_impl->m_bodyCameraToBodyIMU = E::Map<const E::Matrix4f>(imuCharacterization.BodyCameraToBodyIMU.data(), 4, 4).cast<double>();
    }

    UnfilteredIMU::~UnfilteredIMU() = default;

    void UnfilteredIMU::AddSample(const mage::SensorSample& sample)
    {
        if (sample.GetType() == mage::SensorSample::SampleType::Gyrometer ||
            sample.GetType() == mage::SensorSample::SampleType::Accelerometer)
        {
            m_impl->m_samples.insert(sample);
        }
    }

    mage::Pose UnfilteredIMU::GetPose() const
    {
        Eigen::Vector3f position{ m_impl->Position.cast<float>() };
        return mage::Pose(FromQuatAndTrans(m_impl->Orientation.cast<float>(), { position.x(), position.y(), position.z() }));
    }

    void UnfilteredIMU::PredictUpTo(mage::SensorSample::Timestamp timestamp)
    {
        m_impl->PredictUpTo(timestamp);
    }

    void UnfilteredIMU::AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp)
    {
        m_impl->ProcessPose(timestamp, pose);
    }
}
