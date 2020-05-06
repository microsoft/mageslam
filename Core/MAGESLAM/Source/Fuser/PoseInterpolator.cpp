// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "PoseInterpolator.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils/cv.h"
#include <boost/optional.hpp>

#include <arcana/analysis/object_trace.h>
#include <arcana/analysis/data_point.h>

namespace E = Eigen;

namespace mage
{
    using seconds = std::chrono::duration<double>;

    struct alignas(16) PoseInterpolator::Impl
    {
        mage::SensorSample::Timestamp m_gyroTime{};
        mage::SensorSample::Timestamp m_accelTime{};

        mage::Pose m_previousPose;

        std::vector<std::pair<E::Vector3d, mage::SensorSample::Timestamp>, E::aligned_allocator<E::Vector3d>> m_positionHistory;

        Eigen::Vector3d Acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d Velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d Position = Eigen::Vector3d::Zero();

        E::AngleAxisd AngularVelocity = E::AngleAxisd::Identity();
        Eigen::Quaterniond Orientation = Eigen::Quaterniond::Identity();

        void ProcessAngularVelocityUntil(mage::SensorSample::Timestamp ts)
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

        void ProcessLinearVelocityUntil(mage::SensorSample::Timestamp ts)
        {
            if (m_accelTime == mage::SensorSample::Timestamp{})
                m_accelTime = ts;

            seconds dt = ts - m_accelTime;

            Position += Velocity * dt.count();

            m_accelTime = ts;
        }

        void ProcessPose(mage::SensorSample::Timestamp ts, const mage::Pose& pose)
        {
            assert(m_positionHistory.empty() || ts >= m_positionHistory.back().second);

            ProcessAngularVelocityUntil(ts);
            ProcessLinearVelocityUntil(ts);

            auto transform = Decompose(pose.GetInverseViewMatrix());

            Orientation = transform.second.cast<double>();
            Position = ToMap(transform.first).cast<double>();

            if (!m_positionHistory.empty())
            {
                seconds dt = ts - m_positionHistory.back().second;
                auto previous = Decompose(m_previousPose.GetInverseViewMatrix());

                E::Vector3d previousPosition = ToMap(previous.first).cast<double>();

                Velocity = (Position - previousPosition) / dt.count();

                AngularVelocity = (transform.second * previous.second.inverse()).cast<double>();
                AngularVelocity.angle() /= dt.count();
            }

            m_previousPose = pose;

            m_positionHistory.push_back(make_pair(Position, ts));
            if (m_positionHistory.size() == 3)
            {
                auto dt = seconds(m_positionHistory[2].second - m_positionHistory[1].second).count();

                Velocity = (m_positionHistory[2].first - m_positionHistory[1].first) / dt;

                E::Vector3d accel = (m_positionHistory[2].first - 2 * m_positionHistory[1].first + m_positionHistory[0].first) / (dt * dt);

                FIRE_OBJECT_TRACE("IMU Linear Accel.PoseInterpolator", this, (mira::make_data_point<float>(
                    m_positionHistory[1].second,
                    (float)accel.norm()
                )));

                m_positionHistory.erase(m_positionHistory.begin());
            }

            FIRE_OBJECT_TRACE("IMU Linear Velocity.PoseInterpolator", this, (mira::make_data_point<float>(
                ts,
                (float)Velocity.norm()
            )));
        }
    };

    PoseInterpolator::PoseInterpolator()
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}) }
    {}

    PoseInterpolator::~PoseInterpolator() = default;

    mage::Pose PoseInterpolator::GetPose() const
    {
        Eigen::Vector3f position{ m_impl->Position.cast<float>() };
        return mage::Pose(FromQuatAndTrans(m_impl->Orientation.cast<float>(), { position.x(), position.y(), position.z() }));
    }

    void PoseInterpolator::PredictUpTo(mage::SensorSample::Timestamp ts)
    {
        m_impl->ProcessAngularVelocityUntil(ts);
        m_impl->ProcessLinearVelocityUntil(ts);
    }

    void PoseInterpolator::AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp)
    {
        m_impl->ProcessPose(timestamp, pose);
    }
}
