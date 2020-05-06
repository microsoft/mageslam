// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <vector>
#include <gsl/span>
#include <map>

#include <Eigen/Geometry>

namespace mage
{
    struct BundlerParameters
    {
        bool  ArePointsFixed{ false };        // True if map points should not be optimized
    };

    class BundlerLib
    {
    public:
        BundlerLib(const BundlerParameters& bundlerParameters);
        ~BundlerLib();

        void AllocateCameras(size_t count);

        void SetCameraPose(size_t idx,
            Eigen::Map<const Eigen::Vector3f> position,
            Eigen::Map<const Eigen::Matrix3f> orientation,
            Eigen::Map<const Eigen::Vector4f> intrinsics, bool isFixed);

        void FixCameraPose(size_t idx, bool value);

        void AllocateMapPoints(size_t count);
        void SetMapPoint(size_t idx, Eigen::Map<const Eigen::Vector3f> point);

        void AllocateObservations(size_t count);
        void SetObservation(size_t idx, Eigen::Map<const Eigen::Vector2f> position, size_t cameraIndex, size_t mapPointIndex, float informationMatrixScalar);

        void AllocateFixedDistanceConstraints(size_t count);
        void SetFixedDistanceConstraint(size_t idx, size_t cameraIndex1, size_t cameraIndex2, float distance = 1.0f, float weight = 1.0f);

        void AllocateRelativeRotationConstraints(size_t count);
        void SetRelativeRotationConstraint(size_t idx, size_t cameraIndex1, size_t cameraIndex2, const Eigen::Quaternionf& deltaRotation, float weight = 1.0f);

        void AllocateRelativeTransformConstraints(size_t count);
        void SetRelativeTransformConstraint(size_t idx, size_t cameraIndex1, size_t cameraIndex2, Eigen::Map<const Eigen::Vector3f> deltaPosition, const Eigen::Quaternionf& deltaRotation, float weight);

        void SetCurrentLambda(float userLambda);
        float GetCurrentLambda() const;

        // Runs an iteration of the solver for each provided Huber width.
        // Return the average square error.
        float StepBundleAdjustment(gsl::span<const float> huberWidthPerIteration, float maxErrorSquare, std::vector<unsigned int>& outliers);

        void GetPose(size_t idx, Eigen::Map<Eigen::Vector3f> position, Eigen::Map<Eigen::Matrix3f> orientation) const;
        void GetPoint(size_t idx, Eigen::Map<Eigen::Vector3f> position) const;

    private:
        struct Impl;
        const std::unique_ptr<Impl> m_impl;

        // Description of the problem to optimize from the calling code.
        BundlerParameters m_bundlerParameters;
    };
}
