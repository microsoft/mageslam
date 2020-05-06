// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "BundlerLib.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <unordered_set>

#include <g2o/config.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>

#include <array>

using namespace g2o;

namespace mage
{
    class EdgeScaleConstraint : public  BaseMultiEdge<1, number_t>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeScaleConstraint()
        {
            _vertices.resize(2);
            BaseMultiEdge::resize(2);
        }

        void SetWeight(number_t weight)
        {
            m_weight = weight;
        }

        bool read(std::istream& /*is*/) override { return true; }

        bool write(std::ostream& /*os*/) const override { return true; }

        void computeError() override
        {
            const auto v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            const auto v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            Vector3 dt = v2->estimate().translation() - v1->estimate().translation();
            _error[0] = (_measurement - dt.norm())*m_weight;
        }

    protected:
        number_t m_weight = 0;
    };

    class EdgeRotationConstraint : public  BaseMultiEdge<1, Quaternion>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
        EdgeRotationConstraint()
        {
            _vertices.resize(2);
            BaseMultiEdge::resize(2);
        }
    
        bool read(std::istream& /*is*/) override { return true; }
        bool write(std::ostream& /*os*/) const override { return true; }

        void SetWeight(number_t value)
        {
            m_weight = value;
        }
    
        Quaternion GetRelativeRotation() const
        {
            auto v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            auto v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    
            return (v1->estimate().inverse() * v2->estimate()).rotation();
        }
    
        void computeError() override
        {
            _error[0] = GetRelativeRotation().angularDistance(_measurement) * m_weight;
        }
    
    protected:
        number_t m_weight = 0;
    };

    class StepOptimizer : private SparseOptimizer
    {
    public:
        using SparseOptimizer::setVerbose;
        using SparseOptimizer::setAlgorithm;
        using SparseOptimizer::activeEdges;

        bool addParameter(Parameter* p)
        {
            m_dirty = true;
            return SparseOptimizer::addParameter(p);
        }

        bool addVertex(Vertex* v)
        {
            m_dirty = true;
            return SparseOptimizer::addVertex(v);
        }

        bool addEdge(Edge* e)
        {
            m_dirty = true;
            return SparseOptimizer::addEdge(e);
        }

        bool removeEdge(Edge* e)
        {
            m_dirty = true;
            return SparseOptimizer::removeEdge(e);
        }

        void SetCurrentLambda(number_t lambda)
        {
            // The user lambda is checked by the solver on the 0th iteration;
            // consequently, while setting the lambda does NOT actually dirty
            // the optimizer, it does require the iteration to be reset to 0.
            m_iteration = 0;
            static_cast<OptimizationAlgorithmLevenberg*>(_algorithm)->setUserLambdaInit(lambda);
        }

        bool Step()
        {
            if (m_dirty)
            {
                InitializeOptimization();
            }

            if (m_useless || SparseOptimizer::terminate())
                return false;

            preIteration(m_iteration);
            auto result = _algorithm->solve(m_iteration);
            postIteration(m_iteration);

            m_iteration++;

            return result == OptimizationAlgorithm::OK;
        }

    private:
        bool m_dirty{ true };
        bool m_useless{ false };
        int m_iteration{ 0 };

        void InitializeOptimization()
        {
            SparseOptimizer::initializeOptimization();

            m_useless = _ivMap.size() == 0;
            if (!m_useless)
                _algorithm->init();

            m_iteration = 0;
            m_dirty = false;
        }
    };

    struct BundlerLib::Impl
    {
        int currentId;

        const std::unique_ptr<StepOptimizer> Optimizer;
        const std::unique_ptr<OptimizationAlgorithmLevenberg> Algorithm;
        std::vector<g2o::VertexSE3Expmap, Eigen::aligned_allocator<g2o::VertexSE3Expmap>> PoseVertices;
        std::vector<g2o::VertexSBAPointXYZ, Eigen::aligned_allocator<g2o::VertexSBAPointXYZ>> PointVertices;
        std::vector<g2o::EdgeProjectXYZ2UV, Eigen::aligned_allocator<g2o::EdgeProjectXYZ2UV>> ObservationEdges;
        std::vector<g2o::RobustKernelHuber> HuberKernels;
        std::vector<EdgeScaleConstraint> FixedDistanceConstraints;
        std::vector<EdgeRotationConstraint, Eigen::aligned_allocator<EdgeRotationConstraint>> RelativeRotationConstraints;
        std::vector<EdgeSE3Expmap, Eigen::aligned_allocator<EdgeSE3Expmap>> RelativeTransformConstraints;
    };

    BundlerLib::BundlerLib(const BundlerParameters& bundlerParameters)
        : m_impl{ new Impl{
                std::numeric_limits<int>::max() - 1,
                std::make_unique<StepOptimizer>(),
                std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
                    std::make_unique<g2o::BlockSolver_6_3>(
                        std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>()))
            } },
        m_bundlerParameters(bundlerParameters)
    {
        m_impl->Optimizer->setVerbose(false);
        m_impl->Optimizer->setAlgorithm(m_impl->Algorithm.get());
    }

    void BundlerLib::AllocateCameras(size_t count)
    {
        assert(m_impl->PoseVertices.empty() && "can only allocate once");
        m_impl->PoseVertices.resize(count);

        for (size_t i = 0; i < count; ++i)
        {
            m_impl->PoseVertices[i].setId(gsl::narrow<int>(i));
        }
    }

    void BundlerLib::AllocateMapPoints(size_t count)
    {
        m_impl->PointVertices.resize(count);

        for (size_t i = 0; i < count; ++i)
        {
            m_impl->PointVertices[i].setId(--m_impl->currentId);
        }
    }

    void BundlerLib::AllocateObservations(size_t count)
    {
        m_impl->ObservationEdges.resize(count);

        for (size_t i = 0; i < count; ++i)
        {
            m_impl->ObservationEdges[i].setId(--m_impl->currentId);
        }

        m_impl->HuberKernels.resize(count);
    }

    void BundlerLib::AllocateFixedDistanceConstraints(size_t count)
    {
        m_impl->FixedDistanceConstraints.resize(count);

        for (size_t i = 0; i < count; ++i)
        {
            m_impl->FixedDistanceConstraints[i].setId(--m_impl->currentId);
        }
    }

    void BundlerLib::AllocateRelativeRotationConstraints(size_t count)
    {
        m_impl->RelativeRotationConstraints.resize(count);

        for (size_t i = 0; i < count; ++i)
        {
            m_impl->RelativeRotationConstraints[i].setId(--m_impl->currentId);
        }
    }

    void BundlerLib::AllocateRelativeTransformConstraints(size_t count)
    {
        m_impl->RelativeTransformConstraints.resize(count);

        for (size_t i = 0; i < count; ++i)
        {
            m_impl->RelativeTransformConstraints[i].setId(--m_impl->currentId);
        }
    }

    void BundlerLib::SetCameraPose(size_t idx, Eigen::Map<const Eigen::Vector3f> position, Eigen::Map<const Eigen::Matrix3f> orientation, Eigen::Map<const Eigen::Vector4f> intrinsics, bool isFixed)
    {
        auto vertex = &m_impl->PoseVertices[idx];

        // TODO(14118744) remove this allocation as well
        auto cam_params = new g2o::CameraParameters(intrinsics[2], g2o::Vector2(intrinsics[0], intrinsics[1]), 0.0);
        cam_params->setId(gsl::narrow<int>(idx));
        m_impl->Optimizer->addParameter(cam_params);

        vertex->setFixed(isFixed);
        vertex->setEstimate(
            g2o::SE3Quat(Eigen::Quaternionf{ orientation }.normalized().cast<number_t>(), position.cast<number_t>())
        );

        m_impl->Optimizer->addVertex(vertex);
    }

    void BundlerLib::FixCameraPose(size_t idx, bool value)
    {
        m_impl->PoseVertices[idx].setFixed(value);
    }

    void BundlerLib::SetMapPoint(size_t idx, Eigen::Map<const Eigen::Vector3f> point)
    {
        auto vertex = &m_impl->PointVertices[idx];

        vertex->setMarginalized(true);  // Identify these as not-camera-parameters.
        vertex->setEstimate(point.cast<number_t>());
        vertex->setFixed(m_bundlerParameters.ArePointsFixed);

        m_impl->Optimizer->addVertex(vertex);
    }

    void BundlerLib::SetObservation(size_t idx, Eigen::Map<const Eigen::Vector2f> position, size_t cameraIndex, size_t mapPointIndex, float informationMatrixScalar)
    {
        auto edge = &m_impl->ObservationEdges[idx];

        edge->setVertex(0, &m_impl->PointVertices[mapPointIndex]);
        edge->setVertex(1, &m_impl->PoseVertices[cameraIndex]);
        edge->setMeasurement(position.cast<number_t>());
        edge->information() = informationMatrixScalar * Matrix2::Identity();

        edge->setParameterId(0, gsl::narrow_cast<int>(cameraIndex));

        auto kernel = &m_impl->HuberKernels[idx];
        edge->setRobustKernel(kernel);

        m_impl->Optimizer->addEdge(edge);
    }

    void BundlerLib::SetFixedDistanceConstraint(size_t idx, size_t cameraIndex1, size_t cameraIndex2, float distance, float weight)
    {
        auto edge = &m_impl->FixedDistanceConstraints[idx];

        edge->SetWeight(weight);
        edge->setVertex(0, &m_impl->PoseVertices[cameraIndex1]);
        edge->setVertex(1, &m_impl->PoseVertices[cameraIndex2]);
        edge->setInformation(Eigen::Matrix<number_t, 1, 1>::Identity());
        edge->setMeasurement(distance);

        m_impl->Optimizer->addEdge(edge);
    }

    void BundlerLib::SetRelativeRotationConstraint(size_t idx, size_t cameraIndex1, size_t cameraIndex2, const Eigen::Quaternionf& deltaRotation, float weight)
    {
        auto edge = &m_impl->RelativeRotationConstraints[idx];

        edge->SetWeight(weight);
        edge->setVertex(0, &m_impl->PoseVertices[cameraIndex1]);
        edge->setVertex(1, &m_impl->PoseVertices[cameraIndex2]);
        edge->setInformation(Eigen::Matrix<number_t, 1, 1>::Identity());

        edge->setMeasurement(deltaRotation.cast<number_t>());

        m_impl->Optimizer->addEdge(edge);
    }

    void BundlerLib::SetRelativeTransformConstraint(
        size_t idx, size_t cameraIndex1, size_t cameraIndex2, Eigen::Map<const Eigen::Vector3f> deltaPosition, const Eigen::Quaternionf& deltaRotation, float weight)
    {
        auto edge = &m_impl->RelativeTransformConstraints[idx];

        edge->setVertex(0, &m_impl->PoseVertices[cameraIndex1]);
        edge->setVertex(1, &m_impl->PoseVertices[cameraIndex2]);
        edge->setInformation(Eigen::Matrix<number_t, 6, 6>::Identity() * weight);

        edge->setMeasurement({ deltaRotation.cast<number_t>(), deltaPosition.cast<number_t>() });

        m_impl->Optimizer->addEdge(edge);
    }

    BundlerLib::~BundlerLib() = default;

    void BundlerLib::SetCurrentLambda(float userLambda)
    {
        m_impl->Optimizer->SetCurrentLambda(userLambda);
    }

    float BundlerLib::GetCurrentLambda() const
    {
        return static_cast<float>(m_impl->Algorithm->currentLambda());
    }

    float BundlerLib::StepBundleAdjustment(gsl::span<const float> huberWidthPerIteration, float maxErrorSquare, std::vector<unsigned int>& outliers)
    {
        float priorHuberWidth = -1.f;
        for (float huberWidth : huberWidthPerIteration)
        {
            assert(huberWidth >= 0.f && "Huber widths must be nonnegative");
            
            if (huberWidth != priorHuberWidth)
            {
                for (auto& huberKernel : m_impl->HuberKernels)
                {
                    huberKernel.setDelta(huberWidth);
                }
            
                priorHuberWidth = huberWidth;
            }

            if (!m_impl->Optimizer->Step())
                break;
        }

        std::vector<OptimizableGraph::Edge*> edges2Remove;

        int count = 0;
        number_t error = 0;

        for (auto edge : m_impl->Optimizer->activeEdges())
        {
            auto data = edge->errorData();

            number_t sumSquares = 0;

            for (int i = 0; i < edge->dimension(); i++)
            {
                sumSquares += data[i]*data[i];
            }

            // check to see that the point projects in front of the camera (because reprojection error cannot differenciate)
            const auto* vert = static_cast<const g2o::VertexSBAPointXYZ*>(edge->vertex(0));
            const auto* poseVertex = static_cast<const VertexSE3Expmap*>(edge->vertex(1));
            
            SE3Quat worldPose = poseVertex->estimate().inverse();
            g2o::Vector3 worldPos = vert->estimate();
            g2o::Vector3 projectedVec = worldPos - worldPose.translation();

            // now the sign of the dot product of the forward vector from the projection matrix and a vector to the projected point
            // will tell us if the point is in front of the camera or behind it
            const g2o::Vector3 forward = worldPose.rotation() * g2o::Vector3{ 0, 0, 1 };
            const number_t dotProd = projectedVec.dot(forward);

            if (dotProd <= 0)
            {
                edges2Remove.push_back(edge);
            }
            else if (sumSquares > maxErrorSquare)
            {
                edges2Remove.push_back(edge);
            }
            else
            {
                error += sumSquares;
                count++;
            }
        }

        if (!edges2Remove.empty())
        {
            for (auto edge : edges2Remove)
            {
                // only remove point if it is a camera/point edge.
                // TODO(14118778) PERF: don't do a linear search
                auto itr = std::find_if(m_impl->ObservationEdges.begin(), m_impl->ObservationEdges.end(),
                    [edge](const g2o::EdgeProjectXYZ2UV& obs) { return &obs == edge; });

                if (itr != m_impl->ObservationEdges.end())
                {
                    m_impl->Optimizer->removeEdge(edge);
                    outliers.push_back(gsl::narrow<unsigned int>( std::distance(m_impl->ObservationEdges.begin(), itr) ));
                }
            }
        }

        return (float)(error / count);
    }

    namespace
    {
        inline g2o::Quaternion ToQuat(const g2o::Vector4& rotation)
        {
            return g2o::Quaternion{ rotation.eval().data() };
        }
    }

    void BundlerLib::GetPose(size_t idx, Eigen::Map<Eigen::Vector3f> position, Eigen::Map<Eigen::Matrix3f> orientation) const
    {
        const auto& vertex = m_impl->PoseVertices[idx];

        const auto& pose = vertex.estimate();

        position = pose.translation().cast<float>();
        orientation = pose.rotation().normalized().toRotationMatrix().cast<float>();
    }

    void BundlerLib::GetPoint(size_t idx, Eigen::Map<Eigen::Vector3f> position) const
    {
        const auto& vertex = m_impl->PointVertices[idx];
        position = vertex.estimate().cast<float>();
    }
}
