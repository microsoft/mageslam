// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "BundleAdjust.h"

#include "Utils/Logging.h"
#include "Utils/cv.h"
#include "Analysis/DataFlow.h"
#include "Map/MappingMath.h"

#include <mutex>
#include <atomic>
#include <algorithm>
#include <iterator>
#include <memory>
#include <functional>
#include <numeric>

using namespace std;

namespace mage
{
    namespace
    {
        void BuildDataForG2O(BundlerLib& bundler, AdjustableData& input, mira::determinator& determinator)
        {
            SCOPE_TIMER(BundleAdjust::BuildDataForG2O);

            std::map<Id<Keyframe>, size_t> keyframeToIdx;

            // build up the keyframe poses for g2o
            bundler.AllocateCameras(input.Keyframes.size());
            for (unsigned int keyframeIndex = 0; keyframeIndex < input.Keyframes.size(); keyframeIndex++)
            {
                auto& keyframe = input.Keyframes[keyframeIndex];
                auto& pose = keyframe.GetPose();

                keyframeToIdx.emplace(keyframe.GetId(), keyframeIndex);

                // Grab per-keyframe intrinsics.  Note that the keyframe.AnalyzedImage()->GetCalibration() is the "Seed" set of intrinsics that
                // is assumed for the frame based on device-model calibration and focus setting.  keyframe.GetUndistortedIntrinsics() is a mutable copy of
                // these intriniscs that can be optimized by the bundler.
                DETERMINISTIC_CHECK(determinator, pose);
                DETERMINISTIC_CHECK(determinator, keyframe);

                Eigen::Matrix3f rotation;
                cv::cv2eigen(pose.GetRotationMatrix(), rotation);

                bundler.SetCameraPose(
                    keyframeIndex,
                    ToCMap(pose.GetViewSpacePosition()),
                    ToCMap(rotation),
                    ToCMap(keyframe.GetUndistortedIntrinsics().GetCoefficients()),
                    keyframe.IsFixed());
            }

            struct TetherConstraint
            {
                const size_t OriginIdx;
                const size_t DestinationIdx;
                const Tether<Keyframe>& Tether;

                TetherConstraint(size_t originIdx, size_t destinationIdx, const mage::Tether<Keyframe>& tether)
                    : OriginIdx{ originIdx }, DestinationIdx{ destinationIdx }, Tether{ tether }
                {}
            };
            std::vector<TetherConstraint> distanceTethers;
            std::vector<TetherConstraint> rotationTethers;
            std::vector<TetherConstraint> transformTethers;

            for (size_t i = 0; i < input.Keyframes.size(); ++i)
            {
                // Add tethering constraint if keyframe is tethered.
                for (const auto& tether : input.Keyframes[i].GetTethers())
                {
                    switch (tether.Type())
                    {
                    case TetherType::DISTANCE:
                    {
                        auto found = keyframeToIdx.find(tether.OriginId());
                        if (found != keyframeToIdx.end())
                        {
                            distanceTethers.emplace_back(found->second, i, tether);
                        }
                        break;
                    }
                    case TetherType::THREE_DOF:
                    {
                        auto found = keyframeToIdx.find(tether.OriginId());
                        if (found != keyframeToIdx.end())
                        {
                            rotationTethers.emplace_back(found->second, i, tether);
                        }
                        break;
                    }
                    case TetherType::EXTRINSIC:
                    {
                        auto found = keyframeToIdx.find(tether.OriginId());
                        if (found != keyframeToIdx.end())
                        {
                            transformTethers.emplace_back(found->second, i, tether);
                        }
                        break;
                    }
                    default:
                    {
                        assert(false && "not implemented");
                        break;
                    }
                    }
                }
            }

            for (const Id<Keyframe>& id : input.ExternallyTetheredKeyframes)
            {
                size_t idx = keyframeToIdx.find(id)->second;
                input.Keyframes[idx].SetFixed(true);
                bundler.FixCameraPose(idx, true);
            }

            std::map<Id<MapPoint>, size_t> mapPointToIdx;

            bundler.AllocateMapPoints(input.MapPoints.size());
            for (size_t i = 0; i < input.MapPoints.size(); ++i)
            {
                mapPointToIdx.emplace(input.MapPoints[i].GetId(), i);
                bundler.SetMapPoint(i, ToCMap(input.MapPoints[i].GetPosition()));
            }

            bundler.AllocateObservations(input.MapPointAssociations.size());
            for (size_t i = 0; i < input.MapPointAssociations.size(); ++i)
            {
                auto& observation = input.MapPointAssociations[i];
                auto keyFrameIndexItr = keyframeToIdx.find(observation.KeyframeId);
                assert(keyFrameIndexItr != keyframeToIdx.end() && "tried adding an association to a keyframe that isn't in the bundle");

                auto mapPointIndexItr = mapPointToIdx.find(observation.MapPointId);
                assert(mapPointIndexItr != mapPointToIdx.end() && "tried adding an association to a map point that isn't in the bundle");

            float  info = 1.0f;
            // scale by the refinement count so that points with less bundle iterations end up with a smaller information matrix
            // smaller information matrix corresponds to a larger variance in the measurement
            unsigned int refinementCount = input.MapPoints[mapPointIndexItr->second].GetRefinementCount();

            info *= MapPointRefinementConfidence(refinementCount);
            
            bundler.SetObservation(
                i,
                ToCMap(observation.Projection),
                keyFrameIndexItr->second,
                mapPointIndexItr->second,
                info);
        }

            bundler.AllocateFixedDistanceConstraints(distanceTethers.size());
            for (size_t i = 0; i < distanceTethers.size(); ++i)
            {
                auto& constraint = distanceTethers[i];
                bundler.SetFixedDistanceConstraint(i,
                    constraint.OriginIdx,
                    constraint.DestinationIdx,
                    constraint.Tether.Distance(),
                    constraint.Tether.Weight());
            }

            bundler.AllocateRelativeRotationConstraints(rotationTethers.size());
            for (size_t i = 0; i < rotationTethers.size(); ++i)
            {
                auto& constraint = rotationTethers[i];
                bundler.SetRelativeRotationConstraint(i,
                    constraint.OriginIdx,
                    constraint.DestinationIdx,
                    constraint.Tether.Rotation(),
                    constraint.Tether.Weight());
            }

            bundler.AllocateRelativeTransformConstraints(transformTethers.size());
            for (size_t i = 0; i < transformTethers.size(); ++i)
            {
                // Invert the relative transform to match the expectations of the bundler.
                auto& constraint = transformTethers[i];
                auto mat = constraint.Tether.TransformMatrix().inv();
                auto pos = mat.col(3).get_minor<3, 1>(0, 0);
                auto rot = mat.get_minor<3, 3>(0, 0);

                bundler.SetRelativeTransformConstraint(i,
                    constraint.OriginIdx,
                    constraint.DestinationIdx,
                    ToCMap(pos),
                    ToQuat(rot),
                    constraint.Tether.Weight());
            }
        }

        void UpdateData(BundlerLib& bundler, AdjustableData& input)
        {
            SCOPE_TIMER(BundleAdjust::UpdateInputData);

            // update Keyframes
            for (size_t i = 0; i < input.Keyframes.size(); i++)
            {
                // only update the cameras that were allowed to move
                if (!input.Keyframes[i].IsFixed())
                {
                    cv::Point3f position;
                    Eigen::Matrix3f rotation;

                    bundler.GetPose(i, ToMap(position), ToMap(rotation));

                    cv::Matx33f cvmat;
                    cv::eigen2cv(rotation, cvmat);

                    input.Keyframes[i].SetPose(Pose{ position, cvmat });
                }
            }

            // update Map points
            // Important this is done after keyframe updates so the the UpdateMeanViewDirectionAndDistances is set correctly.
            for (unsigned int i = 0; i < input.MapPoints.size(); i++)
            {
                cv::Point3f position;
                bundler.GetPoint(i, ToMap(position));

                input.MapPoints[i].SetPosition(position);
            }
        }

        std::unique_ptr<BundlerLib> MakeBundler(bool fixMapPoints)
        {
            SCOPE_TIMER(BundleAdjust::BuildG2OWrapper);

            BundlerParameters parameters{};
            parameters.ArePointsFixed = fixMapPoints;

            return std::make_unique<BundlerLib>(parameters);
        }
    }

    BundleAdjust::Scheduler::~Scheduler()
    {}

    BundleAdjust::BundleAdjust(AdjustableData& input, mira::determinator& determinator, thread_memory memory)
        : m_input{ input },
        m_determinator{ determinator }
    {
        DATAFLOW(
            DF_INPUT(GetData())
        );

        DETERMINISTIC_CHECK(m_determinator, input);
    }

    std::vector<BundleAdjust::Outlier> BundleAdjust::GetOutliers() const
    {
        std::vector<Outlier> outliers;
        outliers.reserve(m_outliers.size());

        for (auto outlier : m_outliers)
        {
            const auto& assoc = m_input.MapPointAssociations[outlier];
            outliers.emplace_back(assoc.MapPointId, assoc.KeyframeId);
        }

        return outliers;
    }

    std::vector<BundleAdjust::Outlier> BundleAdjust::GetCurrentIterationOutliers() const
    {
        std::vector<BundleAdjust::Outlier> outliers;
        outliers.reserve(m_currentIterationOutliers.size());

        for (auto outlier : m_currentIterationOutliers)
        {
            const auto& assoc = m_input.MapPointAssociations[outlier];
            outliers.emplace_back(assoc.MapPointId, assoc.KeyframeId);
        }

        return outliers;
    }

    int BundleAdjust::RunBundleAdjustment(
        const Scheduler& scheduler,
        float huberWidth,
        float maxOutlierError,
        float maxOutlierErrorScaleFactor,
        float minMeanSquareError,
        bool fixMapPoints,
        uint numStepsPerRun,
        uint numSteps,
        uint minSteps,
        thread_memory memory)
    {
        std::unique_ptr<BundlerLib> bundler = MakeBundler(fixMapPoints);

        SCOPE_TIMER(BundleAdjust::RunBundleAdjustment);

        m_outliers.clear();
        BuildDataForG2O(*bundler, m_input, m_determinator);

        unsigned int count = 0;
        // Makes sure we do at least one pass

        float maxOutlierErrorSquared = maxOutlierError;

        int iteration = 0;

        float residual_error = numeric_limits<float>::max();

        const float outlierScaleSquare = maxOutlierErrorScaleFactor*maxOutlierErrorScaleFactor;
        bool willIterateAgain = false;
        do
        {
            //Step G2O Bundler
            {
                SCOPE_TIMER(BundleAdjust::StepBundleAdjustment::g2o);
                m_currentIterationOutliers.clear();
                vector<float> huberWidths(numStepsPerRun, huberWidth);
                residual_error = bundler->StepBundleAdjustment(huberWidths, maxOutlierErrorSquared, m_currentIterationOutliers);

                DETERMINISTIC_CHECK(m_determinator, m_currentIterationOutliers);
            }

            count += numStepsPerRun;
            maxOutlierErrorSquared *= outlierScaleSquare;

            willIterateAgain = scheduler.ShouldKeepIterating() && count < numSteps && (residual_error > minMeanSquareError || count < minSteps);

            // update the input data with the results of the interation
            {
                SCOPE_TIMER(BundleAdjust::StepBundleAdjustment::UpdateData);

                DATAFLOW(
                    DF_OUTPUT(GetData())
                    DF_OUTPUT(gsl::make_span(GetCurrentIterationOutliers()))
                );

                UpdateData(*bundler, m_input);

                scheduler.OnInputDataUpdated(*this, willIterateAgain);
            }

            m_outliers.insert(m_outliers.end(), m_currentIterationOutliers.begin(), m_currentIterationOutliers.end());

            iteration++;
        } while (scheduler.ShouldKeepIterating() && count < numSteps && (residual_error > minMeanSquareError || count < minSteps));


        {
            SCOPE_TIMER(BundleAdjust::DestroyG2OWrapper);
            bundler.reset();
        }

        return iteration;
    }

    struct BundleAdjustTask::Impl
    {
        AdjustableData& Input;
        mira::determinator& Determinator;
        std::vector<unsigned int> Outliers{};
        std::vector<unsigned int> CurrentIterationOutliers{};
        std::unique_ptr<BundlerLib> Bundler;

        size_t Iterations = 0;
        float MaxOutlierErrorSquared{};

        float ResidualError = std::numeric_limits<float>::max();

        const float OutlierScaleSquare;
        const size_t StepsPerRun;

        Impl(AdjustableData& input, mira::determinator& determinator,
            float maxOutlierError, float maxOutlierScaleFactor, size_t stepsPerRun)
            : Input{ input }, Determinator{ determinator },
            MaxOutlierErrorSquared{ maxOutlierError },
            OutlierScaleSquare{ maxOutlierScaleFactor * maxOutlierScaleFactor },
            StepsPerRun{ stepsPerRun }
        {}

        void Iterate(float huberWidth)
        {
            // Step G2O Bundler
            {
                SCOPE_TIMER(BundleAdjust::StepBundleAdjustment::g2o);
                CurrentIterationOutliers.clear();
                vector<float> huberWidths(StepsPerRun, huberWidth);
                ResidualError = Bundler->StepBundleAdjustment(huberWidths, MaxOutlierErrorSquared, CurrentIterationOutliers);

                DETERMINISTIC_CHECK(Determinator, CurrentIterationOutliers);
            }

            Outliers.insert(Outliers.end(), CurrentIterationOutliers.begin(), CurrentIterationOutliers.end());
            
            Iterations++;

            MaxOutlierErrorSquared *= OutlierScaleSquare;

            // update the input data with the results of the interation
            {
                SCOPE_TIMER(BundleAdjust::StepBundleAdjustment::UpdateData);

                UpdateData(*Bundler, Input);
            }
        }
    };

    BundleAdjustTask::BundleAdjustTask(
        AdjustableData& input,
        mira::determinator& determinator,
        float maxOutlierError,
        float maxOutlierErrorScaleFactor,
        bool fixMapPoints,
        size_t numStepsPerRun)
        : m_impl{ std::make_unique<Impl>(input, determinator, maxOutlierError, maxOutlierErrorScaleFactor, numStepsPerRun) }
    {
        m_impl->Bundler = MakeBundler(fixMapPoints);

        BuildDataForG2O(*m_impl->Bundler, input, determinator);
    }

    void BundleAdjustTask::IterateBundleAdjust(float huberWidth)
    {
        m_impl->Iterate(huberWidth);
    }

    BundleAdjustTask::BundleAdjustTask(std::unique_ptr<Impl> impl)
        :   m_impl{ std::move(impl) }
    {}

    BundleAdjustTask::~BundleAdjustTask() = default;

    size_t BundleAdjustTask::GetTotalSteps() const
    {
        return m_impl->Iterations * m_impl->StepsPerRun;
    }

    size_t BundleAdjustTask::GetIterations() const
    {
        return m_impl->Iterations;
    }

    float BundleAdjustTask::GetResidualError() const
    {
        return m_impl->ResidualError;
    }

    size_t BundleAdjustTask::GetOutlierCount() const
    {
        return m_impl->Outliers.size();
    }

    std::vector<BundleAdjustTask::Outlier> BundleAdjustTask::GetOutliers() const
    {
        std::vector<Outlier> outliers;
        outliers.reserve(m_impl->Outliers.size());

        for (auto outlier : m_impl->Outliers)
        {
            const auto& assoc = m_impl->Input.MapPointAssociations[outlier];
            outliers.emplace_back(assoc.MapPointId, assoc.KeyframeId);
        }

        return outliers;
    }

    std::vector<BundleAdjustTask::Outlier> BundleAdjustTask::GetCurrentIterationOutliers() const
    {
        std::vector<Outlier> outliers;
        outliers.reserve(m_impl->CurrentIterationOutliers.size());

        for (auto outlier : m_impl->CurrentIterationOutliers)
        {
            const auto& assoc = m_impl->Input.MapPointAssociations[outlier];
            outliers.emplace_back(assoc.MapPointId, assoc.KeyframeId);
        }

        return outliers;
    }

    const AdjustableData& BundleAdjustTask::GetData() const
    {
        return m_impl->Input;
    }

    void BundleAdjustTask::SetCurrentLambda(float currentLambda)
    {
        m_impl->Bundler->SetCurrentLambda(currentLambda);
    }

    float BundleAdjustTask::GetCurrentLambda() const
    {
        return m_impl->Bundler->GetCurrentLambda();
    }
}
