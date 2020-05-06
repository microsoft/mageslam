// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <opencv2\core.hpp>
#include <gsl\gsl>

namespace mage
{
    namespace clouds
    {
        void ComputeCharacteristics(
            gsl::span<const cv::Point3f> points,
            std::vector<cv::Vec3f>& normals,
            gsl::span<const size_t> mapIds,
            std::vector<float>& effectiveDissimilarities,
            std::vector<float>& distanceScores,
            std::vector<float>& homogeneityScores,
            const size_t maxN);

        void MollifyNormals(
            gsl::span<const cv::Point3f> points,
            gsl::span<cv::Vec3f> normals,
            const float normalScale,
            const float spatialScale,
            const size_t iterations,
            const size_t maxN);

        void RepositionPointSets(
            gsl::span<cv::Point3f> toReposition,
            gsl::span<cv::Vec3f> normals,
            const float stepi,
            const size_t iterations,
            const float lambda,
            const float solverSigmaS,
            const float mollSigmaS,
            const float mollSigmaN,
            const size_t maxN);

        struct Knn final
        {
            Knn(gsl::span<const cv::Point3f> points);
            Knn(gsl::span<const cv::Vec3f> points);
            ~Knn();

            Knn& operator=(Knn&& other);

            void Query(const cv::Vec3f& point, const size_t N, std::vector<size_t>& indexes) const;

        private:
            struct Impl;
            std::unique_ptr<Impl> m_impl;
        };
    }
}
