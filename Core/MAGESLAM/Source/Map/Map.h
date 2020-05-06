// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <future>
#include <list>
#include <memory>
#include <queue>
#include <vector>
#include <set>
#include <unordered_map>

#include <Eigen/SparseCore>

#include "MageSettings.h"
#include "Data\Data.h"
#include "Data\Types.h"
#include "Mapping\MapPointKeyframeAssociations.h"
#include "Proxies\KeyframeProxy.h"

namespace mage
{
    class Keyframe;
    class KeyframeBuilder;
    class MapPoint;
    class CovisibilityGraph;
    class SpanningTree;

    class Map
    {
    public:
        Map(const CovisibilitySettings& covisSettings, const KeyframeSettings& keyframeSettings, const RelocalizationSettings& relocalizationSettings);
        ~Map();

        Keyframe* AddKeyframe(std::unique_ptr<Keyframe>&& keyframe);
        void RemoveKeyframe(Keyframe* keyframe, thread_memory memory);

        MapPoint* AddMapPoint(std::unique_ptr<MapPoint>&& mapPoint);
        void RemoveMapPoint(Id<MapPoint> mapPoint);

        void Associate(Keyframe* keyframe, MapPoint* mappoint, KeypointDescriptorIndex index);

        void UpdateGraph(const Keyframe* keyframe, thread_memory memory);

        void RemoveAssociation(MapPoint* mappoint, Keyframe* keyframe);
        KeypointDescriptorIndex RemoveAssociation(Keyframe* keyframe, MapPoint* mappoint);

        void RemoveAllAssociations(MapPoint* mapPoint, std::set<const Keyframe*>& modifiedKeyframes);
        void RemoveAllAssociations(Keyframe* keyframe);

        const MapPoint* GetMapPoint(Id<MapPoint> id) const;
        MapPoint* GetMapPoint(Id<MapPoint> id);

        void MergeMapPoints(Id<MapPoint> from, Id<MapPoint> into);

        void UpdateKeyframesFromProxies(
            gsl::span<const KeyframeProxy> consumables,
            const OrbMatcherSettings& mergeSettings,
            std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>& mapPointMerges);

        void UpdateMapPoints(KeyframeProxy& proxy) const;

        void GetSampleOfMapPoints(temp::vector<MapPointProxy>& mapPoints, uint32_t max, uint32_t offset) const;

        bool MapPointExists(const Id<MapPoint> id) const;

        Keyframe* Map::GetKeyframe(const Id<Keyframe> id);
        const Keyframe* Map::GetKeyframe(const Id<Keyframe> id) const;

        size_t GetMapPointsCount() const;
        size_t GetKeyframesCount() const;
        void GetMapPointsAsPositions(std::vector<Position>& positions) const;
        void GetKeyframeViewMatrices(std::vector<Matrix>& viewMatrices) const;
        std::vector<Id<Keyframe>> GetKeyframeIds() const;

        void GetCovisibilityConnectedKeyframes(
            const Id<Keyframe> keyframes,
            temp::unique_vector<Id<Keyframe>>& connectedKeyframes) const;

        void GetCovisibilityConnectedKeyframes(
            const Id<Keyframe> keyframes,
            temp::unique_vector<Id<Keyframe>>& connectedKeyframes,
            unsigned int theta) const;

        void GetTetheredKeyframes(const Id<Keyframe>& keyframeId, temp::unique_vector<Id<Keyframe>>& tetheredKeyframes, const TetherType & type) const;

        void FindExternallyTetheredKeyframesInSet(const std::set<Id<Keyframe>>& keyframes, temp::unique_vector<Id<Keyframe>>& externallyTetheredKeyframes) const;

        void CalculateSharedMapPoints(const Keyframe* keyframe, std::map<const Id<Keyframe>, unsigned int>& sharedMapPoints) const;

        void GetCovisibilityConnectedSubGraphs(loop::set<Id<Keyframe>>&, thread_memory, std::vector<std::vector<Id<Keyframe>>>& output) const;

        void Clear(thread_memory memory);

        void DebugOutput();
        bool ValidSpanningTree();

        template<typename C>
        void IterateKeyframes(const C& iterator) const
        {
            for (const auto& kf : m_keyframes)
                iterator(*kf);
        }

        template<typename C>
        void IterateMapPoints(const C& iterator) const
        {
            for (const auto& mp : m_mapPoints)
                iterator(*mp.second);
        }

        bool AreMapPointsGoodMergeCandidates(const MapPoint*, const MapPoint*, const OrbMatcherSettings&) const;

        void LogSnapshot() const;

    private:
        std::vector<std::unique_ptr<Keyframe>> m_keyframes;
        std::unordered_map<Id<MapPoint>, std::unique_ptr<MapPoint>> m_mapPoints;

        const CovisibilitySettings& m_covisSettings;
        const KeyframeSettings& m_keyframeSettings;
        const RelocalizationSettings& m_relocSettings;

        std::unique_ptr<CovisibilityGraph> m_CovisGraph;
        std::unique_ptr<SpanningTree> m_SpanningTree;

        friend void MapRemoveAllAssociations(Map* map, MapPoint* mapPoint, std::set<const Keyframe*>& modifiedKeyframes);
        // NOT IMPLEMENTED: remove mappoints if local bundle adjustment discards outlier observations in keyframes (PAPER VIB)
    };
}
