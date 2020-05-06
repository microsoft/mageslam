// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Tracking/KeyframeBuilder.h"
#include "Mapping/MapPointKeyframeAssociations.h"
#include "Map/MappingKeyframe.h"
#include "MageSettings.h"

#include <arcana/analysis/determinator.h>
#include <arcana/threading/cancellation.h>
#include <arcana/analysis/introspector.h>

#include <BundlerLib.h>

#include <vector>
#include <tuple>

// forward-declare the friend class
namespace UnitTests
{
    class LocalBundleAdjustmentUnitTests;
}

namespace mage
{
    template <typename T>
    struct TransformConstraint
    {
        T Keyframe0;
        T Keyframe1;
        cv::Vec3f Translation;
        Quaternion Rotation;
        float Weight;

        TransformConstraint(const T& kf0, const T& kf1, const cv::Vec3f& t, const Quaternion& r, float weight)
            : Keyframe0{ kf0 }, Keyframe1{ kf1 }, Translation{ t }, Rotation{ r }, Weight{ weight }
        {}
    };

    struct AdjustableData
    {
        std::vector<MapPointTrackingProxy> MapPoints;
        std::vector<Proxy<Keyframe, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints>> Keyframes;
        std::vector<Id<Keyframe>> ExternallyTetheredKeyframes;
        std::vector<MapPointAssociation> MapPointAssociations;

        void Clear()
        {
            MapPoints.clear();
            Keyframes.clear();
            ExternallyTetheredKeyframes.clear();
            MapPointAssociations.clear();
        }
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const AdjustableData& data)
    {
        intro(
            cereal::make_nvp("MapPoints", data.MapPoints),
            cereal::make_nvp("Keyframes", data.Keyframes),
            cereal::make_nvp("ExternallyTetheredKeyframes", data.ExternallyTetheredKeyframes),
            cereal::make_nvp("MapPointAssociations", data.MapPointAssociations)
        );
    }

    class BundleAdjust
    {
        friend ::UnitTests::LocalBundleAdjustmentUnitTests;

    public:
        using Outlier = std::pair<Id<MapPoint>, Id<Keyframe>>;

        class Scheduler
        {
        public:
            virtual ~Scheduler() = 0;

            /*
                Returns whether or not the bundle adjuster can
                keep iterating.
            */
            bool ShouldKeepIterating() const
            {
                return !m_cancellation->cancelled() && CustomShouldKeepIterating();
            }

            /*
                This method gets called after the model gets
                updated with the new values from the bundle adjust.
                Also notifies the scheduler if it will be iterating again.
            */
            virtual void OnInputDataUpdated(const BundleAdjust&, bool) const
            {}

            /*
                Gets the cancellation token associated with
                the bundle adjust operation.
            */
            const mira::cancellation& cancellation() const
            {
                return *m_cancellation;
            }
        protected:
            virtual bool CustomShouldKeepIterating() const
            {
                return true;
            }

            Scheduler(mira::cancellation::ptr cancellation)
                : m_cancellation{ std::move(cancellation) }
            {}

            Scheduler(const Scheduler& other) = default;
            Scheduler& operator =(const Scheduler& other) = default;

        private:
            mira::cancellation::ptr m_cancellation;
        };

        class NoOpScheduler final : public Scheduler
        {
        public:
            NoOpScheduler()
                : Scheduler{ std::make_shared<mira::cancellation_source>() }
            {}
        };

        BundleAdjust(AdjustableData& input, mira::determinator& determinator, thread_memory memory);

        int RunBundleAdjustment(
            const Scheduler& scheduler,
            const BundleAdjustSettings& settings,
            thread_memory memory)
        {
            return RunBundleAdjustment(
                scheduler,
                settings.HuberWidth,
                settings.MaxOutlierError,
                settings.MaxOutlierErrorScaleFactor,
                settings.MinMeanSquareError,
                false,
                settings.NumStepsPerRun,
                settings.NumSteps,
                settings.MinSteps,
                memory);
        }

        //Returns number of interations take before getting a close enough solution
        int RunBundleAdjustment(
            const Scheduler& scheduler,
            const BundleAdjustSettings& settings,
            bool fixMapPoints,
            thread_memory memory)
        {
            return RunBundleAdjustment(
                scheduler,
                settings.HuberWidth,
                settings.MaxOutlierError,
                settings.MaxOutlierErrorScaleFactor,
                settings.MinMeanSquareError,
                fixMapPoints,
                settings.NumStepsPerRun,
                settings.NumSteps,
                settings.MinSteps,
                memory);
        }

        //Returns number of interations take before getting a close enough solution
        int RunBundleAdjustment(
            const Scheduler& scheduler,
            float huberWidth,
            float maxOutlierError,
            float maxOutlierErrorScaleFactor,
            float minMeanSquareError,
            bool fixMapPoints,
            uint numStepsPerRun,
            uint numSteps,
            uint minSteps,
            thread_memory memory);

        std::vector<Outlier> GetOutliers() const;

        std::vector<Outlier> GetCurrentIterationOutliers() const;

        const AdjustableData& GetData() const
        {
            return m_input;
        }

    private :
        AdjustableData& m_input;

        std::vector<unsigned int> m_outliers;
        std::vector<unsigned int> m_currentIterationOutliers;

        mira::determinator& m_determinator;
    };

    class BundleAdjustTask
    {
        friend ::UnitTests::LocalBundleAdjustmentUnitTests;

    public:
        using Outlier = std::pair<Id<MapPoint>, Id<Keyframe>>;

        BundleAdjustTask(
            AdjustableData& input,
            mira::determinator& determinator,
            float maxOutlierError,
            float maxOutlierErrorScaleFactor,
            bool fixMapPoints,
            size_t numStepsPerRun);

        BundleAdjustTask(BundleAdjustTask&&) = default;
        BundleAdjustTask& operator=(BundleAdjustTask&&) = default;

        ~BundleAdjustTask();

        void IterateBundleAdjust(float huberWidth);

        size_t GetTotalSteps() const;
        size_t GetIterations() const;

        float GetResidualError() const;

        size_t GetOutlierCount() const;

        std::vector<Outlier> GetOutliers() const;
        std::vector<Outlier> GetCurrentIterationOutliers() const;

        const AdjustableData& GetData() const;

        void SetCurrentLambda(float currentLambda);
        float GetCurrentLambda() const;

    private:
        struct Impl;

        BundleAdjustTask(std::unique_ptr<Impl> impl);
        std::unique_ptr<Impl> m_impl;
    };
}
