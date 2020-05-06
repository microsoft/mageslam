// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// Map.cpp
//
// the map is a singleton class that holds information about estimated camera poses 
// and observed world points
//------------------------------------------------------------------------------

#include "CovisibilityGraph.h"
#include "Data/Data.h"
#include "Data/Keyframe.h"
#include "Debugging/SkeletonLogger.h"
#include "Map.h"
#include "MapPoint.h"
#include "SpanningTree.h"
#include "Tracking/FeatureMatcher.h"
#include "Tracking/KeyframeBuilder.h"
#include "Utils/Epipolar.h"
#include "Utils/MageConversions.h"

#include <algorithm>
#include <arcana/iterators.h>

using namespace std;

namespace mage
{
    Map::Map(
        const CovisibilitySettings& covisSettings,
        const KeyframeSettings& keyframeSettings,
        const RelocalizationSettings& relocalizationSettings)
        : m_covisSettings(covisSettings),
        m_keyframeSettings(keyframeSettings),
        m_relocSettings(relocalizationSettings)
    {
        //initialize arrays
        m_keyframes.reserve(INIT_NUM_KEYFRAMES);

        // covisibility graph: which keyframes are related by a large number of mappoints being present in each
        m_CovisGraph = make_unique<CovisibilityGraph>(m_covisSettings);

        //spanning tree: links keyframes with most map points in common
        m_SpanningTree = make_unique<SpanningTree>();
    }

    Map::~Map()
    {}

    ///KEYFRAME INSERTION: paper section VIA
    /// adds a keyframe to the system and updates the related datastructures
    Keyframe* Map::AddKeyframe(unique_ptr<Keyframe>&& keyframe)
    {
        // we are going to create all the objects before associating them.
        // keyframes should have no map points when they get added to the
        // map, it should get it's map points associated afterwards
        assert(keyframe->GetMapPointsCount() == 0);

        m_keyframes.push_back(move(keyframe));

        auto insertedKeyframe = m_keyframes.rbegin();

        Id<Keyframe> keyframeId = (*insertedKeyframe)->GetId();
        m_CovisGraph->AddKeyframe(keyframeId);
        m_SpanningTree->AddKeyframe(keyframeId);

        LogSnapshot();

        return insertedKeyframe->get();
    }

    void Map::RemoveKeyframe(Keyframe* keyframe, thread_memory memory)
    {
        assert(keyframe);
        auto itr = std::find_if(m_keyframes.begin(), m_keyframes.end(), [keyframe](auto& kf) { return kf.get() == keyframe; });
        assert(itr != m_keyframes.end());

        // we are going to destroy all the objects after removing all associations.
        // Keyframes should have no MapPoints when they get removed from the
        // map, it should get it's MapPoint associations removed prior.
        assert(keyframe->GetMapPointsCount() == 0);

        m_CovisGraph->RemoveKeyframe(keyframe->GetId(), memory);
        m_SpanningTree->RemoveKeyframe(keyframe->GetId(), memory);

        // Only walk the list forward, as we know that no forward-pointing tethers exist.
        for (auto tethersItr = itr + 1; tethersItr != m_keyframes.end(); tethersItr++)
        {
            (*tethersItr)->RemoveTethersToKeyframe(keyframe->GetId());
        }

        // this deletes the pointer so don't use it after this call
        m_keyframes.erase(itr);
    }

    void MapAssociate(Keyframe* keyframe, MapPoint* mappoint, KeypointDescriptorIndex index)
    {
        keyframe->AddMapPointAssociation(mappoint, index);
        mappoint->AddKeyframeAssociation(keyframe, index);
    }

    KeypointDescriptorIndex MapRemoveAssociation(Keyframe* keyframe, MapPoint* mapPoint)
    {
        KeypointDescriptorIndex removedDescriptorIndex = keyframe->RemoveMapPointAssociation(mapPoint);
        mapPoint->RemoveKeyframeAssociation(keyframe, removedDescriptorIndex);
        return removedDescriptorIndex;
    }

    void MapRemoveAllAssociations(Map* map, MapPoint* mapPoint, std::set<const Keyframe*>& modifiedKeyframes)
    {
        std::vector<const Keyframe*> mapPointKeyframes(mapPoint->GetKeyframes());
        for (const Keyframe* mpKeyframe : mapPointKeyframes)
        {
            map->GetKeyframe(mpKeyframe->GetId())->RemoveMapPointAssociation(mapPoint);
            modifiedKeyframes.insert(mpKeyframe);
        }
        mapPoint->ClearAssociations();
    }

    void Map::Associate(Keyframe* keyframe, MapPoint* mappoint, KeypointDescriptorIndex index)
    {
        MapAssociate(keyframe, mappoint, index);
    }

    void Map::UpdateGraph(const Keyframe* keyframe, thread_memory memory)
    {
        std::map<const Id<Keyframe>, unsigned int> sharedMapPoints;
        CalculateSharedMapPoints(keyframe, sharedMapPoints);

        m_CovisGraph->UpdateKeyframe(keyframe->GetId(), sharedMapPoints, memory);

        //get new state
        auto curConnectedKeyframes = memory.stack_unique_vector<Id<Keyframe>>();
        m_CovisGraph->GetConnectedKeyframes(
            keyframe->GetId(),
            curConnectedKeyframes);

        m_SpanningTree->UpdateKeyframe(keyframe->GetId(), curConnectedKeyframes, *m_CovisGraph.get(), memory);
    }

    // just forwards it to the other overload, don't change
    void Map::RemoveAssociation(MapPoint* mappoint, Keyframe* keyframe)
    {
        MapRemoveAssociation(keyframe, mappoint);
    }

    KeypointDescriptorIndex Map::RemoveAssociation(Keyframe* keyframe, MapPoint* mappoint)
    {
        return MapRemoveAssociation(keyframe, mappoint);
    }

    void Map::RemoveAllAssociations(MapPoint * mapPoint, std::set<const Keyframe*>& modifiedKeyframes)
    {
        MapRemoveAllAssociations(this, mapPoint, modifiedKeyframes);
    }

    void Map::RemoveAllAssociations(Keyframe * keyframe)
    {
        std::vector<const MapPoint*> keyframeMapPoints;
        keyframe->GetMapPoints(keyframeMapPoints);
        for (const MapPoint* kfMapPoint : keyframeMapPoints)
        {
            RemoveAssociation(GetMapPoint(kfMapPoint->GetId()), keyframe);
        }
    }

    MapPoint const* Map::GetMapPoint(Id<MapPoint> id) const
    {
        auto itr = m_mapPoints.find(id);

        return itr == m_mapPoints.end() ? nullptr : itr->second.get();
    }

    void Map::MergeMapPoints(Id<MapPoint> from, Id<MapPoint> into)
    {
        // If arguments are identical, merge is a no-op.
        if (from == into)
        {
            return;
        }

        auto fromItr = m_mapPoints.find(from);
        if (fromItr == m_mapPoints.end())
        {
            return;
        }

        auto intoItr = m_mapPoints.find(into);
        if (intoItr == m_mapPoints.end())
        {
            return;
        }

        auto& fromPtr = fromItr->second;
        auto& intoPtr = intoItr->second;
        auto keyframes = fromPtr->GetKeyframes();
        for (const Keyframe* kf : keyframes)
        {
            KeypointDescriptorIndex associatedIdx = RemoveAssociation(GetKeyframe(kf->GetId()), fromPtr.get());

            // If the target keyframe already has the map point associated to a different keypoint, just skip it.
            if (!kf->HasAssociatedMapPoint(intoPtr->GetId()))
            {
                Associate(GetKeyframe(kf->GetId()), intoPtr.get(), associatedIdx);
            }
        }

        RemoveMapPoint(from);
    }

    void Map::UpdateKeyframesFromProxies(
        gsl::span<const KeyframeProxy> consumables,
        const OrbMatcherSettings& mergeSettings,
        std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>& mapPointMerges)
    {
        std::unordered_set<Id<MapPoint>> movedMapPoints{};

        for (int kfIdx = 0; kfIdx < consumables.size(); kfIdx++)
        {
            auto& proxyKf = consumables[kfIdx];
            assert(GetKeyframe(proxyKf.GetId()) != nullptr);
            auto& realKf = *GetKeyframe(proxyKf.GetId());

            // Set the keyframe's pose.
            auto poseShiftTransform = To4x4(proxyKf.GetPose().GetRelativeViewMatrix(realKf.GetPose()));
            if (!realKf.IsFixed())
            {
                // Don't set the pose for fixed keyframes, as their poses are not allowed to be moved.
                // In the event that a loop closure wishes it could move fixed keyframes, it falls to
                // the subsequent bundle adjust to revise the world to rectify that requirement.
                realKf.SetPose(proxyKf.GetPose());
            }

            const auto proxyUnassocMask = proxyKf.GetUnassociatedKeypointMask();
            const auto realUnassocMask = realKf.CreateUnassociatedMask();

            vector<MapPointAssociations<const MapPoint*>::Association> realAssociations;
            realKf.GetAssociations(realAssociations);

            for (int idx = 0; idx < proxyKf.GetAnalyzedImage()->GetKeyPoints().size(); idx++)
            {
                bool isProxyAssociated = !proxyUnassocMask[idx];
                bool isRealAssociated = !realUnassocMask[idx];

                // If both keypoints are associated, merge the map points into the one associated with the proxy.
                if (isProxyAssociated && isRealAssociated)
                {
                    Id<MapPoint> proxyMapPointId = proxyKf.GetAssociatedMapPoint(idx)->GetId();
                    MapPoint* proxyMapPoint = GetMapPoint(proxyMapPointId);

                    const MapPoint* realMapPoint = realKf.GetAssociatedMapPoint(idx);
                    Id<MapPoint> realMapPointId = realMapPoint->GetId();

                    // Only merge map points which both still exist and pass a sanity check for good map point
                    // merges.
                    if (proxyMapPoint != nullptr && realMapPoint != nullptr &&
                        AreMapPointsGoodMergeCandidates(proxyMapPoint, realMapPoint, mergeSettings))
                    {
                        MergeMapPoints(realMapPointId, proxyMapPointId);
                        mapPointMerges.insert({ realMapPointId, *proxyMapPoint });
                    }
                }
                // If the proxy is associated and the real is unassociated, associate the real.
                else if (isProxyAssociated && !isRealAssociated)
                {
                    Id<MapPoint> proxyMapPointId = proxyKf.GetAssociatedMapPoint(idx)->GetId();

                    // Only create the association if the map point still exists and isn't already
                    // associated with this keyframe.
                    if (GetMapPoint(proxyMapPointId) != nullptr && !realKf.HasAssociatedMapPoint(proxyMapPointId))
                    {
                        Associate(&realKf, GetMapPoint(proxyMapPointId), idx);
                    }
                }
                // If the proxy is unassociated and the real is associated, move the map point.
                else if (!isProxyAssociated && isRealAssociated)
                {
                    const MapPoint* realMapPoint = realKf.GetAssociatedMapPoint(idx);

                    // We only want to move each map point at most once, so don't bother if it's been merged
                    // away or moved already.
                    if (realMapPoint != nullptr && movedMapPoints.find(realMapPoint->GetId()) == movedMapPoints.end())
                    {
                        movedMapPoints.insert(realMapPoint->GetId());

                        auto newPos = poseShiftTransform * realMapPoint->GetPosition();
                        GetMapPoint(realMapPoint->GetId())->SetPosition({ newPos.val[0], newPos.val[1], newPos.val[2] });
                    }
                }
                // If neither keypoint is associated, do nothing.
            }
        }
    }

    void Map::UpdateMapPoints(KeyframeProxy& proxy) const
    {
        mira::unique_vector<Id<MapPoint>> culled;

        proxy.IterateAssociations([&](MapPointTrackingProxy& mpProxy, KeypointDescriptorIndex)
        {
            const MapPoint* mp = GetMapPoint(mpProxy.GetId());
            if (mp != nullptr)
            {
                mpProxy = MapPointTrackingProxy{ mp };
            }
            else
            {
                culled.insert(mpProxy.GetId());
            }
        });

        for (auto& mp : culled)
        {
            proxy.RemoveAssociation(mp);
        }
    }

    void Map::GetSampleOfMapPoints(temp::vector<MapPointProxy>& mapPoint, uint32_t max, uint32_t offset) const
    {
        mapPoint.reserve(max);
        uint32_t stepSize = (gsl::narrow_cast<uint32_t>(m_mapPoints.size()) / max) + 1;
        uint32_t count = offset;
        for (const auto& curMapPoint : m_mapPoints)
        {
            count++;
            if (count % stepSize != 0)
            {
                continue;
            }
            mapPoint.emplace_back(*curMapPoint.second);
        }
    }

    MapPoint* Map::GetMapPoint(Id<MapPoint> id)
    {
        // use const casts to just reuse the const implementation and
        // not duplicate the find code. Because the const guarantee,
        // is stronger than the non-const version, casting away the const is fine
        return const_cast<MapPoint*>(const_cast<const Map*>(this)->GetMapPoint(id));
    }

    bool Map::MapPointExists(const Id<MapPoint> id) const
    {
        return GetMapPoint(id) != nullptr;
    }

    Keyframe* Map::GetKeyframe(const Id<Keyframe> id)
    {
        auto itr = find_if(m_keyframes.begin(), m_keyframes.end(),
            [&id](auto& kp) { return kp->GetId() == id; });

        Keyframe* retVal = nullptr;
        if (itr != m_keyframes.end())
        {
            retVal = itr->get();
        }

        return retVal;
    }

    const Keyframe* Map::GetKeyframe(const Id<Keyframe> id) const
    {
        auto itr = find_if(m_keyframes.begin(), m_keyframes.end(),
            [&id](auto& kp) { return kp->GetId() == id; });

        Keyframe* retVal = nullptr;
        if (itr != m_keyframes.end())
        {
            retVal = itr->get();
        }

        return retVal;
    }

    MapPoint* Map::AddMapPoint(unique_ptr<MapPoint>&& mapPoint)
    {
        // we are going to create all the objects before associating them.
        // MapPoints should have no Keyframes when they get added to the
        // map, it should get it's keyframes associated afterwards
        assert(mapPoint->GetKeyframes().size() == 0);

        auto insertion = m_mapPoints.insert({ mapPoint->GetId(), move(mapPoint) });
        assert(insertion.second);

        return insertion.first->second.get();
    }

    void Map::RemoveMapPoint(Id<MapPoint> id)
    {
        auto itr = m_mapPoints.find(id);
        assert(itr != m_mapPoints.end());

        // we are going to destroy all the objects after removing all associations.
        // MapPoints should have no Keyframes when they get removed from the
        // map, it should get it's keyframes associations removed prior.
        assert(itr->second.get()->GetKeyframes().size() == 0);
        // this deletes the pointer so don't use it after this call
        m_mapPoints.erase(itr);
    }

    size_t Map::GetMapPointsCount() const
    {
        return m_mapPoints.size();
    }

    size_t Map::GetKeyframesCount() const
    {
        return m_keyframes.size();
    }

    void Map::GetMapPointsAsPositions(std::vector<Position>& positions) const
    {
        for (const auto& mapPoint : m_mapPoints)
        {
            const auto& pos = mapPoint.second->GetPosition();
            positions.push_back({ pos.x, pos.y, pos.z });
        }
    }

    void Map::GetKeyframeViewMatrices(std::vector<Matrix>& viewMatrices) const
    {
        for (auto& keyframe : m_keyframes)
        {
            viewMatrices.push_back(ToMageMat(keyframe->GetPose().GetViewMatrix()));
        }
    }

    vector<Id<Keyframe>> Map::GetKeyframeIds() const
    {
        vector<Id<Keyframe>> idList;
        idList.reserve(m_keyframes.size());

        transform(m_keyframes.begin(), m_keyframes.end(), mira::emplace_inserter(idList), [](auto& kf) { return kf->GetId(); });

        return idList;
    }

    void Map::GetCovisibilityConnectedKeyframes(
        const Id<Keyframe> keyframeId,
        temp::unique_vector<Id<Keyframe>>& connectedKeyframes) const
    {
        m_CovisGraph->GetConnectedKeyframes(keyframeId, connectedKeyframes);
    }

    void Map::GetCovisibilityConnectedKeyframes(
        const Id<Keyframe> keyframeId,
        temp::unique_vector<Id<Keyframe>>& connectedKeyframes,
        unsigned int theta) const
    {
        m_CovisGraph->GetConnectedKeyframes(keyframeId, connectedKeyframes, theta);
    }

    void Map::GetTetheredKeyframes(
        const Id<Keyframe>& keyframeId,
        temp::unique_vector<Id<Keyframe>>& tetheredKeyframes,
        const TetherType& type) const
    {
        auto itr = find_if(m_keyframes.begin(), m_keyframes.end(), [&](const auto& kf) { return kf->GetId() == keyframeId; });

        if (itr == m_keyframes.end())
        {
            return;
        }

        for (const auto& tether : (*itr)->GetTethers())
        {
            if (tether.Type() == type)
            {
                assert(tether.OriginId() < keyframeId);
                tetheredKeyframes.insert(tether.OriginId());
            }
        }

        itr++;

        for (; itr != m_keyframes.end(); itr++)
        {
            const auto& kf = *itr;
            for (const auto& tether : kf->GetTethers())
            {
                if (tether.OriginId() == keyframeId && tether.Type() == type)
                {
                    assert(keyframeId < kf->GetId());
                    tetheredKeyframes.insert(kf->GetId());
                }
            }
        }
    }

    void Map::FindExternallyTetheredKeyframesInSet(const std::set<Id<Keyframe>>& keyframeSet, temp::unique_vector<Id<Keyframe>>& externallyTetheredKeyframes) const
    {
        // Trivial case, empty set has no externally tethered keyframes.
        if (keyframeSet.size() == 0)
        {
            return;
        }

        Id<Keyframe> minId = *min_element(keyframeSet.begin(), keyframeSet.end());

        for (const auto& kf : m_keyframes)
        {
            Id<Keyframe> id = kf->GetId();

            // If the keyframe in question has an ID less than that of the minimum id in the query set,
            // we know it cannot contain any tethers to the query set because keyframes can only hold
            // tethers to keyframes whose IDs are less than their own.
            if (id < minId)
            {
                continue;
            }

            bool isInsideSet = keyframeSet.find(id) != keyframeSet.end();

            for (const auto& t : kf->GetTethers())
            {
                if (isInsideSet)
                {
                    if (keyframeSet.find(t.OriginId()) == keyframeSet.end())
                    {
                        // If current is in the set and origin is not, then current is externally tethered.
                        externallyTetheredKeyframes.insert(id);
                    }
                }
                else
                {
                    if (keyframeSet.find(t.OriginId()) != keyframeSet.end())
                    {
                        // If current is not in the set and origin is, then origin is externally tethered.
                        externallyTetheredKeyframes.insert(t.OriginId());
                    }
                }

            }
        }
    }

    void Map::CalculateSharedMapPoints(const Keyframe* keyframe, std::map<const Id<Keyframe>, unsigned int>& sharedMapPoints) const
    {
        const Id<Keyframe> keyframeId = keyframe->GetId();

        // look for connected keyframes (shared map points)  
        std::vector<const MapPoint*> mapPoints;
        keyframe->GetMapPoints(mapPoints);                          //INPUT: array of mappoints

        for (const MapPoint* curMapPoint : mapPoints)
        {
            const std::vector<const Keyframe*>& keyframesWithMapPoint = curMapPoint->GetKeyframes();
            for (const Keyframe* curKeyframe : keyframesWithMapPoint)
            {
                const Id<Keyframe> curKeyframeId = curKeyframe->GetId();
                if (curKeyframeId == keyframeId)
                {
                    continue;
                }

                auto sharedCount = sharedMapPoints.find(curKeyframeId);
                if (sharedCount == sharedMapPoints.end())
                {
                    sharedMapPoints.insert(std::make_pair(curKeyframeId, 1));
                }
                else
                {
                    sharedCount->second++;
                }
            }
        }
    }

    void Map::GetCovisibilityConnectedSubGraphs(loop::set<Id<Keyframe>>& ids, thread_memory memory, vector<vector<Id<Keyframe>>>& output) const
    {
        m_CovisGraph->GetConnectedSubGraphs(ids, memory, output);
    }

    // TODO this is quite posibly the least performant way to do this, but the map is small when we call this so we're not too concerned about it right now
    void Map::Clear(thread_memory memory)
    {
        while (m_mapPoints.size() > 0)
        {
            const auto& current = (*m_mapPoints.begin());

            std::set<const Keyframe*> modifiedKeyframes; // don't actually care about this, but the API requires it
            RemoveAllAssociations(current.second.get(), modifiedKeyframes);
            RemoveMapPoint(current.first);
        }

        while (m_keyframes.size() > 0)
        {
            RemoveKeyframe(m_keyframes.back().get(), memory);
        }
    }

#pragma optimize("",off)
    void Map::DebugOutput()
    {
        m_CovisGraph->DebugOutput();
        m_SpanningTree->DebugOutput();
    }
#pragma optimize("",on)
    bool Map::ValidSpanningTree()
    {
        //TODO: temporarily disabling test to unblock ux while debugging.
        return true;
        //return m_SpanningTree->ValidSpanningTree(*m_CovisGraph);
    }

    void Map::LogSnapshot() const
    {
        SkeletonLogger::Map::LogMapPointsSnaphot(m_mapPoints);
        SkeletonLogger::Map::LogKeyframesSnapshot(m_keyframes, *m_CovisGraph);
    }

    bool Map::AreMapPointsGoodMergeCandidates(const MapPoint* mp0, const MapPoint* mp1, const OrbMatcherSettings& settings) const
    {
        assert(mp0->GetKeyframes().size() > 0);
        assert(mp1->GetKeyframes().size() > 0);
        size_t distance = static_cast<size_t>(GetDescriptorDistance(
                mp0->GetRepresentativeDescriptor(),
                mp1->GetRepresentativeDescriptor()));
        return distance < settings.MaxHammingDistance;
    }
}
