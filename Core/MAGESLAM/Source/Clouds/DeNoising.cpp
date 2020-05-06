// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "DeNoising.h"

#include <opencv2\core.hpp>

#include "Utils\Logging.h"
#include <arcana\threading\parallel.h>

#include <unordered_map>
#include <tuple>

// TODO:  feed back to Boost that 4457 is firing
#pragma warning (push)
#pragma warning (disable:4457)
#include <boost/geometry/strategies/strategies.hpp>
#include <boost\geometry\index\rtree.hpp>
#pragma warning (pop)

#include <tuple>

namespace geo = boost::geometry;

namespace mage
{
    namespace clouds
    {
        using rtree_point = geo::model::point<float, 3, geo::cs::cartesian>;
        using rtree_value = std::ptrdiff_t;

        struct indexable
        {
            using result_type = rtree_point;

            static_assert(sizeof(rtree_point) == sizeof(cv::Point3f), "types should be identical");

            const rtree_point& operator()(const rtree_value& idx) const
            {
                return reinterpret_cast<const rtree_point&>(points[idx]);
            }

            gsl::span<const cv::Point3f> points;
        };

        using rtree = geo::index::rtree<rtree_value, geo::index::rstar<12>, indexable>;

        struct Knn::Impl
        {
            rtree Tree;
        };

        Knn::Knn(gsl::span<const cv::Vec3f> points)
            : Knn({ reinterpret_cast<const cv::Point3f*>(points.data()), points.size() })
        {
            static_assert(sizeof(cv::Vec3f) == sizeof(cv::Point3f), "types should have identical layouts");
        }

        Knn::Knn(gsl::span<const cv::Point3f> points)
        {
            std::vector<size_t> indexes(points.size());
            std::iota(indexes.begin(), indexes.end(), 0);

            m_impl = std::make_unique<Impl>(Impl{ rtree{ indexes.begin(), indexes.end(), rtree::parameters_type{}, indexable{ points } } });
        }

        Knn& Knn::operator=(Knn&& other)
        {
            m_impl = std::move(other.m_impl);
            return *this;
        }

        Knn::~Knn()
        {}

        void Knn::Query(const cv::Vec3f& vi, const size_t N, std::vector<size_t>& indexes) const
        {
            indexes.resize(N);
            size_t count = m_impl->Tree.query(boost::geometry::index::nearest(rtree_point{ vi[0], vi[1], vi[2] }, gsl::narrow_cast<unsigned int>(N)), indexes.begin());
            indexes.resize(count);
        }

        cv::Vec3f Averages(gsl::span<const cv::Vec3f> elements)
        {
            cv::Vec3f average = std::accumulate(elements.begin(), elements.end(), cv::Vec3f{ 0, 0, 0 }) / gsl::narrow_cast<int>(elements.size());
            return average;
        }

        cv::Vec3f Variances(gsl::span<const cv::Vec3f> elements)
        {
            auto avg = Averages(elements);

            cv::Vec3f variances{ 0, 0, 0 };

            for (ptrdiff_t i = 0; i < elements.size(); ++i)
            {
                cv::Vec3f del = elements[i] - avg;

                for(int c = 0; c < 3; ++c)
                    variances[c] += del[c] * del[c];
            }

            return variances / gsl::narrow_cast<int>(elements.size());
        }

        cv::Vec3f Medians(gsl::span<const cv::Vec3f> elements)
        {
            cv::Vec3f medians;

            std::vector<float> componentValues(elements.size());

            for (int c = 0; c < 3; ++c)
            {
                std::transform(elements.begin(), elements.end(), componentValues.begin(), [c](const auto& pt) { return pt(c); });

                auto nth = componentValues.begin() + componentValues.size() / 2;

                std::nth_element(componentValues.begin(), nth, componentValues.end());

                medians[c] = *nth;
            }

            return medians;
        }

        void ComputeCharacteristics(
            gsl::span<const cv::Point3f> points,
            std::vector<cv::Vec3f>& normals,
            gsl::span<const size_t> mapIds,
            std::vector<float>& effectiveDissimilarities,
            std::vector<float>& distanceScores,
            std::vector<float>& homogeneityScores,
            const size_t maxN)
        {
            if (normals.size() != (size_t)points.size())
            {
                normals.clear();
                normals.resize(points.size());
            }

            effectiveDissimilarities.clear();
            effectiveDissimilarities.resize(points.size());

            distanceScores.clear();
            distanceScores.resize(points.size());

            homogeneityScores.clear();
            homogeneityScores.resize(points.size());

            const size_t N = std::min(maxN, (size_t)points.size());

            const Knn knn{ points };

            // compute all the normals
            mira::parallel(points.size(), 8, [&points, &normals, &knn, indexes = std::vector<size_t>{}, Ni = std::vector<cv::Vec3f>{}, perps = std::vector<cv::Vec3f>{}, N](size_t i) mutable
            {
                const cv::Vec3f vi = points[i];

                knn.Query(vi, N + 1, indexes);
                indexes.erase(std::find(indexes.begin(), indexes.end(), i));

                Ni.resize(indexes.size());
                std::transform(indexes.begin(), indexes.end(), Ni.begin(), [points](auto idx) { return points[idx]; });

                const cv::Vec3f mu = Medians(Ni);

                perps.resize(Ni.size());
                std::transform(Ni.begin(), Ni.end(), perps.begin(), [vi, mu](const cv::Vec3f& vj)
                {
                    float invWij = sqrt(cv::normL2Sqr(vi.val, vj.val, 3));
                    if (invWij < 0.0001f) invWij = 1; // TODO what's a good ratio for an identical point?
                
                    return (1 / invWij) * (cv::Vec3f{ vj } - mu);
                });

                const cv::Mat pcaInputMat(gsl::narrow_cast<int>(perps.size()), 3, CV_32FC1, perps.data());

                const cv::PCA pca{ pcaInputMat, cv::Vec3f::zeros().t(), cv::PCA::Flags::DATA_AS_ROW };

                const auto minel = std::min_element(pca.eigenvalues.begin<float>(), pca.eigenvalues.end<float>());
                const auto index = std::distance(pca.eigenvalues.begin<float>(), minel);

                cv::Vec3f normal{ &pca.eigenvectors.at<float>(gsl::narrow_cast<int>(index), 0) };

                if (normal.dot(normals[i]) < 0)
                {
                    normal *= -1;
                }

                normals[i] = normal;
            });

            mira::parallel(points.size(), 8, [&points, &normals, &knn, &mapIds, &homogeneityScores, &effectiveDissimilarities, &distanceScores, indexes = std::vector<size_t>{}, distances = std::vector<float>{}, N](size_t i) mutable
            {
                const cv::Vec3f vi = points[i];

                knn.Query(vi, N + 1, indexes);
                indexes.erase(std::find(indexes.begin(), indexes.end(), i));

                // compute homogeneity score
                if (!mapIds.empty())
                {
                    size_t same = std::count_if(indexes.begin(), indexes.end(), [&](const size_t j)
                    {
                        return mapIds[i] == mapIds[j];
                    });
                    homogeneityScores[i] = same / (float)N;
                }

                // compute the effective dissimilarites for the point
                {
                    float effectiveDissimilarity = 0.0f;

                    for (size_t k = 0; k < indexes.size(); ++k)
                    {
                        const cv::Point3f& vk = points[indexes[k]];
                        const cv::Vec3f& nk = normals[indexes[k]];

                        const cv::Vec3f vik = vi - cv::Vec3f{ vk };

                        const cv::Vec3f vikParallel = nk * vik.dot(nk);
                        const cv::Vec3f vikPerp = vik - vikParallel;

                        const float dissimilarity = cv::normL2Sqr<float, float>(vikParallel.val, 3) / (cv::normL2Sqr<float, float>(vikPerp.val, 3) + 0.0001f);
                        effectiveDissimilarity += dissimilarity;
                    }

                    effectiveDissimilarities[i] = effectiveDissimilarity / indexes.size();
                }

                // compute the distance based outlier detection
                {
                    distances.resize(indexes.size());
                    std::transform(indexes.begin(), indexes.end(), distances.begin(), [vi, points](const size_t& k)
                    {
                        return cv::normL2Sqr<float, float>(vi.val, &points[k].x, 3);
                    });

                    const auto nth = distances.begin() + distances.size() / 2;

                    std::nth_element(distances.begin(), nth, distances.end());

                    distanceScores[i] = sqrt(*nth);
                }
            });
        }

        void MollifyNormals(
            gsl::span<const cv::Point3f> points,
            gsl::span<cv::Vec3f> normals,
            const float normalScale,
            const float spatialScale,
            const size_t iterations,
            const size_t maxN)
        {
            std::vector<size_t> indexes(points.size());
            std::iota(indexes.begin(), indexes.end(), 0);

            size_t N = std::min(maxN, (size_t)points.size());

            Knn knn{ points };

            std::vector<cv::Vec3f> mollified{ normals.begin(), normals.end() };

            for (size_t iteration = 0; iteration < iterations; ++iteration)
            {
                for (ptrdiff_t i = 0; i < points.size(); ++i)
                {
                    knn.Query(points[i], N + 1, indexes);
                    indexes.erase(find(indexes.begin(), indexes.end(), static_cast<size_t>(i)));

                    cv::Vec3f vi = points[i];
                    cv::Vec3f ni = normals[i];

                    cv::Vec3f summed = ni;
                    for (size_t j : indexes)
                    {
                        cv::Vec3f vj = points[j];
                        cv::Vec3f nj = normals[j];

                        float norms = cv::normL2Sqr<float, float>(ni.val, nj.val, 3) / (normalScale * normalScale);
                        float spats = cv::normL2Sqr<float, float>(vi.val, vj.val, 3) / (spatialScale * spatialScale);

                        float Oij = std::expf(-(norms + spats));
                        summed += Oij * nj;
                    }

                    mollified[i] = cv::normalize(summed);
                }

                for (ptrdiff_t i = 0; i < points.size(); ++i)
                    normals[i] = mollified[i];
            }
        }

        void MollifyNormalsInternal(gsl::span<const cv::Vec3f> points, gsl::span<cv::Vec3f> inOutNormals, gsl::span<const std::vector<size_t>> neighbourhoods, const float normalScale, const float spatialScale)
        {
            const std::vector<cv::Vec3f> normals{ inOutNormals.begin(), inOutNormals.end() };

            for (ptrdiff_t i = 0; i < points.size(); ++i)
            {
                const cv::Vec3f& vi = points[i];
                const cv::Vec3f& ni = normals[i];

                cv::Vec3f summed = ni;
                for (size_t j : neighbourhoods[i])
                {
                    const cv::Vec3f& vj = points[j];
                    const cv::Vec3f& nj = normals[j];

                    float norms = cv::normL2Sqr<float, float>(ni.val, nj.val, 3) / normalScale;
                    float spats = cv::normL2Sqr<float, float>(vi.val, vj.val, 3) / spatialScale;

                    float Oij = std::expf(-(norms + spats));
                    summed += Oij * nj;
                }

                inOutNormals[i] = cv::normalize(summed);
            }
        }
        
        cv::Vec3f LambdaSumDerivative(const cv::Vec3f& original, const cv::Vec3f& modified, float lambda)
        {
            return 2 * lambda * (modified - original);
        };

        float Tau(const cv::Vec3f& vi, const cv::Vec3f& vj, float sigmaSqr)
        {
            float norm2 = cv::normL2Sqr<float, float>(vi.val, vj.val, 3);
            return std::exp(-norm2 / sigmaSqr);
        };

        cv::Vec3f TauDerivedOverVi(const cv::Vec3f& vi, const cv::Vec3f& vj, float sigmaSqr)
        {
            float norm2 = cv::normL2Sqr<float, float>(vi.val, vj.val, 3);
            return (2 / sigmaSqr) * (vj - vi) * std::exp(-norm2 / sigmaSqr);
        };

        cv::Vec3f TauDerivedOverVj(const cv::Vec3f& vi, const cv::Vec3f& vj, float sigmaSqr)
        {
            float norm2 = cv::normL2Sqr<float, float>(vi.val, vj.val, 3);

            //when differentiated by j the sign is flipped
            return -(2 / sigmaSqr) * (vj - vi) * std::exp(-norm2 / sigmaSqr);
        };

        struct
        {
            std::vector<float> SumTijs;
            std::vector<cv::Vec3f> SumTijDis;
            std::vector<cv::Vec3f> SumGammaNormalDerivedOverVi;
        } caches;

        float Gamma(const size_t& vid, const cv::Vec3f& vi, const cv::Vec3f& vj, float sigmaSqr)
        {
            float tij = Tau(vi, vj, sigmaSqr);

            float sumtij = caches.SumTijs[vid];

            return tij / sumtij;
        };

        cv::Vec3f GammaDerivedOverVi(const size_t& vid, const cv::Vec3f& vi, const cv::Vec3f& vj, float sigmaSqr)
        {
            cv::Vec3f tijdi = TauDerivedOverVi(vi, vj, sigmaSqr);
            float tij = Tau(vi, vj, sigmaSqr);

            float sumtij = caches.SumTijs[vid];

            cv::Vec3f sumtijdi = caches.SumTijDis[vid];

            return (tijdi * sumtij - tij * sumtijdi) / (sumtij * sumtij);
        };

        cv::Vec3f GammaDerivedOverVj(const size_t& vid, const cv::Vec3f& vi, const cv::Vec3f& vj, float sigmaSqr)
        {
            cv::Vec3f tijdj = TauDerivedOverVj(vi, vj, sigmaSqr);

            float tij = Tau(vi, vj, sigmaSqr);

            float sumtij = caches.SumTijs[vid];

            return (tijdj * sumtij - tij * tijdj) / (sumtij * sumtij);
        };

        cv::Vec3f GammaDerivedOverVjn(const size_t& vid, const cv::Vec3f& vi, const cv::Vec3f& vj, const cv::Vec3f& vjn, float sigmaSqr)
        {
            float tij = Tau(vi, vj, sigmaSqr);
            cv::Vec3f dtijn = TauDerivedOverVj(vi, vjn, sigmaSqr);

            float sumtij = caches.SumTijs[vid];

            return (-tij / (sumtij * sumtij)) * dtijn;
        };

        float GammaNormal(const size_t& id, const cv::Vec3f& vi, const cv::Vec3f& ni, const cv::Vec3f& vj, float sigmaSqr)
        {
            float gam = Gamma(id, vi, vj, sigmaSqr);
            float dot = ni.dot(vi - vj);
            return gam * dot * dot; // TODO Not certain if the dot should be squared or not
        };

        cv::Vec3f GammaNormalDerivedOverVi(const size_t& id, const cv::Vec3f& vi, const cv::Vec3f& ni, const cv::Vec3f& vj, float sigmaSqr)
        {
            float gam = Gamma(id, vi, vj, sigmaSqr);
            cv::Vec3f gamdix = GammaDerivedOverVi(id, vi, vj, sigmaSqr);

            float dot = ni.dot(vi - vj);
            return 2 * gam * ni * dot + gamdix * dot * dot;
        };

        cv::Vec3f GammaNormalDerivedOverVj(const size_t& id, const cv::Vec3f& vi, const cv::Vec3f& ni, const cv::Vec3f& vj, float sigmaSqr)
        {
            float gam = Gamma(id, vi, vj, sigmaSqr);
            cv::Vec3f gamdjx = GammaDerivedOverVj(id, vi, vj, sigmaSqr);

            float dot = ni.dot(vi - vj);
            return -2 * gam * ni * dot + gamdjx * dot * dot;
        };

        cv::Vec3f GammaNormalDerivedOverVjn(const size_t& id, const cv::Vec3f& vi, const cv::Vec3f& ni, const cv::Vec3f& vj, const cv::Vec3f& vjn, float sigmaSqr)
        {
            cv::Vec3f gamdjnx = GammaDerivedOverVjn(id, vi, vj, vjn, sigmaSqr);

            float dot = ni.dot(vi - vj);
            return gamdjnx * dot * dot;
        };

        cv::Vec3f GammaNormalsDerivedOverVi(
            gsl::span<const cv::Vec3f> points,
            gsl::span<const cv::Vec3f> normals,
            gsl::span<const std::vector<size_t>> neighbourhoods,
            const Knn& knn,
            const cv::Vec3f& realVi,
            size_t realViIndex,
            float sigmaSqr,
            const size_t N)
        {
            cv::Vec3f sum{ 0, 0, 0 };

            std::vector<size_t> searched;
            knn.Query(realVi, 2 * N, searched);

            for (size_t i : searched)
            {
                if (i == realViIndex)
                {
                    sum += caches.SumGammaNormalDerivedOverVi[i];
                }
                else
                {
                    auto& vi = points[i];
                    auto& ni = normals[i];
                    auto& neighborhood = neighbourhoods[i];

                    auto realVj = find_if(neighborhood.begin(), neighborhood.end(), [&](size_t ni) { return realViIndex == ni; });
                    if (realVj != neighborhood.end())
                    {
                        for (size_t j : neighborhood)
                        {
                            if (j == realViIndex)
                            {
                                sum += GammaNormalDerivedOverVj(i, vi, ni, points[j], sigmaSqr);
                            }
                            else
                            {
                                auto realIInNJ = find_if(neighbourhoods[j].begin(), neighbourhoods[j].end(), [&](size_t ni) { return realViIndex == ni; });
                                if (realIInNJ != neighbourhoods[j].end())
                                {
                                    sum += GammaNormalDerivedOverVjn(i, vi, ni, points[j], realVi, sigmaSqr);
                                }
                            }
                        }
                    }
                }
            }

            return sum;
        };

        std::vector<std::vector<size_t>> InternalPrimeCaches(gsl::span<cv::Point3f> points, gsl::span<cv::Vec3f> normals, const float sigmaSqr, const size_t maxN)
        {
            const size_t N = std::min(maxN, (size_t)points.size());

            Knn knn{ points };
            std::vector<std::vector<size_t>> neighbourhoods(points.size());

            mira::parallel(points.size(), 8, [&neighbourhoods, &knn, points, N](size_t i)
            {
                auto& neighbourhood = neighbourhoods[i];

                knn.Query(points[i], N + 1, neighbourhood);
                neighbourhood.erase(find(neighbourhood.begin(), neighbourhood.end(), i));
            });

            caches.SumTijs.clear();
            caches.SumTijs.resize(points.size());

            caches.SumTijDis.clear();
            caches.SumTijDis.resize(points.size());

            caches.SumGammaNormalDerivedOverVi.clear();
            caches.SumGammaNormalDerivedOverVi.resize(points.size());

            mira::parallel(points.size(), 8, [&neighbourhoods, points, sigmaSqr, normals](size_t i)
            {
                float sumtij = 0;
                cv::Vec3f sumtijdi{ 0, 0, 0 };
                for (size_t j : neighbourhoods[i])
                {
                    sumtij += Tau(points[i], points[j], sigmaSqr);
                    sumtijdi += TauDerivedOverVi(points[i], points[j], sigmaSqr);
                }
                caches.SumTijs[i] = abs(sumtij * sumtij) < std::numeric_limits<float>::epsilon() ? (std::signbit(sumtij) ? -1 : 1) : sumtij;
                caches.SumTijDis[i] = sumtijdi;

                cv::Vec3f sumgammanormalderivedovervi{ 0, 0, 0 };
                for (size_t j : neighbourhoods[i])
                {
                    cv::Vec3f val = GammaNormalDerivedOverVi(i, points[i], normals[i], points[j], sigmaSqr);
                    assert(val == val);
                    sumgammanormalderivedovervi += val;
                }
                caches.SumGammaNormalDerivedOverVi[i] = sumgammanormalderivedovervi;
            });

            return neighbourhoods;
        }

        void RepositionPointSets(
            gsl::span<cv::Point3f> toReposition,
            gsl::span<cv::Vec3f> normals,
            const float stepi,
            const size_t iterations,
            const float lambda,
            const float solverSigmaS,
            const float mollSigmaS,
            const float mollSigmaN,
            const size_t maxN)
        {
            std::vector<size_t> indexes;

            const size_t N = std::min(maxN, (size_t)toReposition.size());

            std::vector<cv::Vec3f> initialPoints{ toReposition.begin(), toReposition.end() };
            std::vector<cv::Vec3f> points = initialPoints;
            std::vector<cv::Vec3f> gradients(points.size());

            std::vector<std::vector<size_t>> neighbourhoods(points.size());

            for (size_t iteration = 0; iteration < iterations; ++iteration)
            {
                LogMessage(L"Starting Iteration");

                LogMessage(L"Computing Neighbourhoods");

                Knn knn{ points };

                mira::parallel(points.size(), 8, [&neighbourhoods, &knn, &points, N](size_t i)
                {
                    auto& neighbourhood = neighbourhoods[i];

                    knn.Query(points[i], N + 1, neighbourhood);
                    auto self = find(neighbourhood.begin(), neighbourhood.end(), i);
                    if (self != neighbourhood.end())
                    {
                        neighbourhood.erase(self);
                    }
                });

                LogMessage(L"Mollifying Normals");

                MollifyNormalsInternal(points, normals, neighbourhoods, mollSigmaN * mollSigmaN, mollSigmaS * mollSigmaN);

                LogMessage(L"Computing Neighborhood Sums");

                caches.SumTijs.clear();
                caches.SumTijs.resize(points.size());

                caches.SumTijDis.clear();
                caches.SumTijDis.resize(points.size());

                caches.SumGammaNormalDerivedOverVi.clear();
                caches.SumGammaNormalDerivedOverVi.resize(points.size());

                const float sigmaSqr = solverSigmaS * solverSigmaS;

                mira::parallel(points.size(), 8, [&neighbourhoods, &points, sigmaSqr, normals](size_t i)
                {
                    float sumtij = 0;
                    cv::Vec3f sumtijdi{ 0, 0, 0 };
                    for (size_t j : neighbourhoods[i])
                    {
                        sumtij += Tau(points[i], points[j], sigmaSqr);
                        sumtijdi += TauDerivedOverVi(points[i], points[j], sigmaSqr);
                    }
                    caches.SumTijs[i] = abs(sumtij * sumtij) < std::numeric_limits<float>::epsilon() ? (std::signbit(sumtij) ? -1 : 1) : sumtij;
                    caches.SumTijDis[i] = sumtijdi;

                    cv::Vec3f sumgammanormalderivedovervi{ 0, 0, 0 };
                    for (size_t j : neighbourhoods[i])
                    {
                        cv::Vec3f val = GammaNormalDerivedOverVi(i, points[i], normals[i], points[j], sigmaSqr);
                        assert(val == val);
                        sumgammanormalderivedovervi += val;
                    }
                    caches.SumGammaNormalDerivedOverVi[i] = sumgammanormalderivedovervi;
                });

                LogMessage(L"Computing Gradients");

                mira::parallel(points.size(), 8, [&gradients, &points, &neighbourhoods, &knn, &initialPoints, sigmaSqr, normals, N, lambda](size_t i)
                {
                    gradients[i] = GammaNormalsDerivedOverVi(points, normals, neighbourhoods, knn, points[i], i, sigmaSqr, N) + LambdaSumDerivative(initialPoints[i], points[i], lambda);
                    assert(gradients[i] == gradients[i]);
                });

                for (size_t i = 0; i < points.size(); ++i)
                {
                    points[i] = points[i] - stepi * gradients[i];
                }
            }

            for (size_t i = 0; i < points.size(); ++i)
                toReposition[i] = points[i];

            caches = {}; // release memory used by the compute caches
        }
    }
}
