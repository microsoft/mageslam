// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "PoseGraphOptimizer.h"

#include <g2o/config.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <gsl/gsl>

using namespace g2o;
using namespace std;

namespace g2o
{
    EdgeSim3::EdgeSim3() {}

    bool EdgeSim3::write(ostream&) const
    {
        return false;
    }

    bool EdgeSim3::read(istream&)
    {
        return false;
    }

    VertexSim3Expmap::VertexSim3Expmap() {}

    bool VertexSim3Expmap::write(ostream&) const
    {
        return false;
    }

    bool VertexSim3Expmap::read(istream& /*is*/)
    {
        return false;
    }
}

namespace mage
{
    struct PoseGraphOptimizer::Impl
    {
        bool graphIsDirty; // True if InitializeOptimization needs to be called before the next iteration.
        int currentId;

        const unique_ptr<SparseOptimizer> Optimizer;
        const unique_ptr<OptimizationAlgorithm> Algorithm;
        vector<VertexSim3Expmap, Eigen::aligned_allocator<VertexSim3Expmap>> Variables;
        vector<EdgeSim3, Eigen::aligned_allocator<EdgeSim3>> Constraints;
    };

    PoseGraphOptimizer::PoseGraphOptimizer()
        : m_impl{ new Impl{
        true,
        std::numeric_limits<int>::max() - 1,
        std::make_unique<g2o::SparseOptimizer>(),
        std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
            std::make_unique<g2o::BlockSolver_7_3>(
                std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_7_3::PoseMatrixType>>()))
    } }
    {
        m_impl->Optimizer->setVerbose(false);
        m_impl->Optimizer->setAlgorithm(m_impl->Algorithm.get());
    }

    PoseGraphOptimizer::~PoseGraphOptimizer() = default;

    size_t PoseGraphOptimizer::AddVariable(const Sim3Transform& t, bool isFixed)
    {
        m_impl->Variables.emplace_back();
        auto v = &m_impl->Variables.back();
        v->setId(gsl::narrow<int>(m_impl->Variables.size() + m_impl->Constraints.size() - 1));

        v->setFixed(isFixed);
        v->_fix_scale = false;
        v->setEstimate(
            g2o::Sim3(g2o::Quaternion{ t.Rotation.cast<number_t>() }.normalized(), g2o::Vector3{ t.Position(0), t.Position(1), t.Position(2) }, t.Scale)
        );

        return m_impl->Variables.size() - 1;
    }

    size_t PoseGraphOptimizer::AddConstraint(const Sim3Transform& t, size_t fromIdx, size_t toIdx)
    {
        m_impl->Constraints.emplace_back();
        auto e = &m_impl->Constraints.back();
        e->setId(gsl::narrow<int>(m_impl->Variables.size() + m_impl->Constraints.size() - 1));

        e->setVertex(0, &m_impl->Variables[fromIdx]);
        e->setVertex(1, &m_impl->Variables[toIdx]);
        e->setInformation(Eigen::Matrix<number_t, 7, 7>::Identity());
        e->setMeasurement(
            g2o::Sim3(g2o::Quaternion{ t.Rotation.cast<number_t>() }.normalized(), g2o::Vector3{ t.Position(0), t.Position(1), t.Position(2) }, t.Scale)
        );

        return m_impl->Constraints.size() - 1;
    }

    void PoseGraphOptimizer::StepBundleAdjustment(int numInterations)
    {
        // If the graph topology has changed since we last initialized the optimization problem, we need to re-initialize.
        if (m_impl->graphIsDirty)
        {
            for (auto& v : m_impl->Variables)
            {
                m_impl->Optimizer->addVertex(&v);
            }

            for (auto& e : m_impl->Constraints)
            {
                m_impl->Optimizer->addEdge(&e);
            }

            m_impl->Optimizer->setVerbose(true);
            m_impl->Optimizer->initializeOptimization();
            m_impl->graphIsDirty = false;
        }

        m_impl->Optimizer->optimize(numInterations);
    }

    void PoseGraphOptimizer::GetVariable(size_t idx, Sim3Transform& transform) const
    {
        auto& vertex = m_impl->Variables[idx];

        auto& pose = vertex.estimate();

        transform.Position = pose.translation().cast<float>();
        transform.Rotation = pose.rotation().normalized().toRotationMatrix().cast<float>();

        transform.Scale = static_cast<float>(pose.scale());
    }
}
