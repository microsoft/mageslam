// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <memory>

#include <Eigen/Geometry>

namespace g2o
{
    class VertexSim3Expmap;
    class EdgeSim3;
}

namespace mage
{
    class PoseGraphOptimizer
    {
    public:
        struct Sim3Transform
        {
            Eigen::Vector3f Position;
            Eigen::Matrix3f Rotation;
            float Scale = 1.f;

            Sim3Transform Inverse() const
            {
                Eigen::Matrix3f r = Rotation.transpose();
                return {
                    r * (Position / -Scale),
                    r,
                    1 / Scale
                };
            }

            Sim3Transform operator *(const Sim3Transform& other) const
            {
                Sim3Transform ret;
                ret.Rotation = Rotation * other.Rotation;
                ret.Position = Scale * (Rotation * other.Position) + Position;
                ret.Scale = Scale * other.Scale;
                return ret;
            }

            Eigen::Vector3f GetScaledPosition() const
            {
                return Position / Scale;
            }
        };

        PoseGraphOptimizer();
        ~PoseGraphOptimizer();

        size_t AddVariable(const Sim3Transform&, bool isFixed = false);
        size_t AddConstraint(const Sim3Transform&, size_t fromIdx, size_t toIdx);

        void StepBundleAdjustment(int numInterations);

        void GetVariable(size_t idx, Sim3Transform& transform) const;

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
