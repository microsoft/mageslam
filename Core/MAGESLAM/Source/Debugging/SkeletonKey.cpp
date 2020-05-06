// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "SkeletonKey.h"

#include "SkeletonData.h"
#include "SkeletonLogger.h"
#include "Utils\MageConversions.h"

using namespace std;

namespace mage
{
    std::unique_ptr<SkeletonKey> SkeletonKey::Craft(const MAGESlam& mage)
    {
        return unique_ptr<SkeletonKey>{ new SkeletonKey{mage} };
    }

    struct SkeletonKey::Impl
    {
        SkeletonData data;
    };

    extern function<SkeletonData (const MAGESlam&)> g_backdoor;

    SkeletonKey::SkeletonKey(const MAGESlam& mage)
        : m_impl{ make_unique<Impl>() }
    {
        m_impl->data = g_backdoor(mage);
    }

    void SkeletonKey::GetMapPoints(std::vector<Position>& positions) const
    {
        return m_impl->data.map->GetMapPointsAsPositions(positions);
    }

    void  SkeletonKey::GetViewMatrices(std::vector<Matrix>& viewMatrices) const
    {
        std::vector<Pose> poses;
        m_impl->data.history->DebugGetAllPoses(poses);

        viewMatrices.clear();
        viewMatrices.reserve(poses.size());

        transform(poses.begin(), poses.end(), back_inserter(viewMatrices),
            [](const Pose& pose)
        {
            return ToMageMat(pose.GetViewMatrix());
        });
    }

    void  SkeletonKey::GetKeyframeViewMatrices(std::vector<Matrix>& viewMatrices) const
    {
        return m_impl->data.map->GetKeyframeViewMatrices(viewMatrices);
    }

    void SkeletonKey::SetSkeletonLoggingLevel(SkeletonLoggerLevel level) const
    {
        SkeletonLogger::SetLevel(level);
    }

    FuserMode SkeletonKey::GetFuserMode()
    {
        return m_impl->data.fuser->GetMode();
    }

    void SkeletonKey::AddIntrospector(Introspector& inspector)
    {
        m_impl->data.sharedData->Introspection.AddIntrospector(inspector);
    }

    SkeletonKey::~SkeletonKey()
    {}

    struct FossilizedSkeletonKey::Impl
    {
        FossilizedSkeletonData data;
    };

    extern function<FossilizedSkeletonData(const MAGESlam::FossilizedMap&)> g_fossilizedBackdoor;

    FossilizedSkeletonKey::FossilizedSkeletonKey(const MAGESlam::FossilizedMap& map)
        : m_impl{ make_unique<Impl>() }
    {
        m_impl->data = g_fossilizedBackdoor(map);
    }

    FossilizedSkeletonKey::~FossilizedSkeletonKey()
    {}

    std::unique_ptr<FossilizedSkeletonKey> FossilizedSkeletonKey::Craft(const MAGESlam::FossilizedMap& map)
    {
        return unique_ptr<FossilizedSkeletonKey>{ new FossilizedSkeletonKey{ map } };
    }

    void FossilizedSkeletonKey::GetViewMatrices(std::vector<Matrix>& viewMatrices) const
    {
        std::vector<Pose> poses;
        m_impl->data.History->DebugGetAllPoses(poses);

        viewMatrices.clear();
        viewMatrices.reserve(poses.size());

        transform(poses.begin(), poses.end(), back_inserter(viewMatrices),
            [](const Pose& pose)
            {
                return ToMageMat(pose.GetViewMatrix());
            });
    }

    void FossilizedSkeletonKey::GetMapPoints(std::vector<Position>& positions) const
    {
        positions = m_impl->data.MapPoints;
    }
}
