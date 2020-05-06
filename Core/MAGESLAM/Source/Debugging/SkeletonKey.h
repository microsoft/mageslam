// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <experimental\filesystem>

#include "MageSlam.h"
#include "Introspector.h"

namespace mage
{
    enum class SkeletonLoggerLevel
    {
        Off = 0b0,
        Initialization = 0b1,
        Tracking = 0b10,
        Mapping = 0b100,
        Threads = Initialization | Tracking | Mapping,
        Image = 0b1000,
        Model = 0b10000,                   
        GloballyAdjustedPoses = Tracking | Image,
        Full = 0b11111,
    };

    struct InitializationData;

    class SkeletonKey
    {
    public:
        static std::unique_ptr<SkeletonKey> Craft(const MAGESlam& mage);

        ~SkeletonKey();

        void GetMapPoints(std::vector<Position>& positions) const;
        void GetViewMatrices(std::vector<Matrix>& viewMatrices) const;
        void GetKeyframeViewMatrices(std::vector<Matrix>& viewMatrices) const;
        void SetSkeletonLoggingLevel(SkeletonLoggerLevel level) const;

        void AddIntrospector(Introspector& inspector);

        void BeginIMULogging(const std::experimental::filesystem::path& imuFile);
        void EndIMULogging();
        FuserMode GetFuserMode();

    private:
        SkeletonKey(const MAGESlam& mage);

        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };

    class FossilizedSkeletonKey
    {
    public:
        static std::unique_ptr<FossilizedSkeletonKey> Craft(const MAGESlam::FossilizedMap& map);

        ~FossilizedSkeletonKey();

        void GetViewMatrices(std::vector<Matrix>& viewMatrices) const;
        void FossilizedSkeletonKey::GetMapPoints(std::vector<Position>& positions) const;

    private:
        FossilizedSkeletonKey(const MAGESlam::FossilizedMap& mage);

        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
