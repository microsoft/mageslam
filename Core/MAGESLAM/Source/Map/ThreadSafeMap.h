// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Map.h"

#include "MapPoint.h"
#include "MappingKeyframe.h"
#include "InitializationData.h"

#include "Tracking\KeyframeBuilder.h"

#include "Mapping\MapPointKeyframeAssociations.h"

#include "BoW\OnlineBow.h"
#include "BoW\BaseFeatureMatcher.h"

#include "BundleAdjustment\BundleAdjust.h"

#include "Data\Types.h"
#include "Data\Pose.h"
#include "Data\Data.h"
#include "Data\MapState.h"

#include "Utils\historical_queue.h"
#include "Utils\collection.h"

#include <arcana\threading\blocking_concurrent_queue.h>

#include <gsl\gsl>

#include <shared_mutex>
#include <vector>
#include <set>


// forward-declare the friend class method for accessing the root map in unit tests
namespace UnitTests
{
    class TestHelperFunctions;
}

namespace mage
{
    class ThreadSafeMap
    {
        friend class ::UnitTests::TestHelperFunctions;

    public:
        ThreadSafeMap(
            const MageSlamSettings& settings, BaseBow& bagOfWords);

        ~ThreadSafeMap();

        void InitializeMap(const InitializationData& initializationData, thread_memory memory);

        void GetConnectedMapPoints(
            const collection<Proxy<MapPoint>>& points,
            thread_memory memory,
            loop::vector<KeyframeReprojection>& repro) const;

        // returns a sampling of mappoints
        void GetSampleOfMapPoints(temp::vector<MapPointProxy>& mapPoints, uint32_t max, uint32_t offset = 0) const;

        /*
        Inserts a new keyframe and returns all the keyframes that
        are connected to this one by the covisibility graph.
        */
        Proxy<Keyframe, proxy::Image> InsertKeyframe(
            const std::shared_ptr<const KeyframeBuilder>& kb,
            thread_memory memory);

        /*
        Inserts a new keyframe and returns all the keyframes that
        are connected to this one by the covisibility graph.
        */
        Proxy<Keyframe, proxy::Image> InsertKeyframe(
            const std::shared_ptr<const KeyframeBuilder>& keyframe,
            const gsl::span<MapPointAssociations<MapPointTrackingProxy>::Association>& extraAssociation,
            thread_memory memory);

        std::unique_ptr<KeyframeProxy> GetKeyFrameProxy(const Id<Keyframe>& kId) const;

        /*
        Merge information from a set of posited keyframes into the actual keyframes.
        */
        void UpdateKeyframesFromProxies(
            gsl::span<const KeyframeProxy> proxies,
            const OrbMatcherSettings& mergeSettings,
            std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>& mapPointMerges,
            thread_memory memory);

        /*
        Updates the state of the map points in the KeyframeProxy based on the current
        information in the map.
        */
        void UpdateMapPoints(KeyframeProxy& proxy) const;

        /*
        Builds a global bundle adjust dataset of everything in the map.
        NOTE: Fixes the position of fixed and distance-tethered keyframes to preserve some semblance
        of consistent scale.
        */
        void BuildGlobalBundleAdjustData(AdjustableData& data) const;

        /*
        Retrieves keyframes similar to the image provided.
        */
        void FindSimilarKeyframes(const std::shared_ptr<const AnalyzedImage>& image, std::vector<KeyframeProxy>& output) const;

        void FindNonCovisibleSimilarKeyframeClusters(const KeyframeProxy& proxy, thread_memory memory, std::vector<std::vector<KeyframeProxy>>& output) const;

        /*
        Culls map points based on requirements related to number of keyframes they are visibile in and the results found in tracking
        Takes a list of points that have failed to appear where predicted during tracking to remove as well
        */
        void CullRecentMapPoints(
            const Id<Keyframe>& ki,
            const mira::unique_vector<Id<MapPoint>>& recentFailedMapPoints,
            thread_memory memory);

        /*
        Return the collection of all the map points
        */
        void GetMapPointsAsPositions(std::vector<Position>& positions) const;

        /*
        Return the number of map points in the map
        */
        size_t GetMapPointsCount() const;

        /*
        Return the number of keyframes in the map
        */
        size_t GetKeyframesCount() const;

        /*
        Return the collection of all the keyframes
        */
        void GetKeyframeViewMatrices(std::vector<Matrix>& viewMatrices) const;

        /*
        Creates the map points passed in and returns the keyframes that
        were added to the frames connected to Ki through the covis graph
        */
        std::vector<MappingKeyframe> CreateMapPoints(
            const Id<Keyframe>& keyframeId,
            const gsl::span<const MapPointKeyframeAssociations >& toCreate,
            thread_memory memory);

        /*
        Returns all the map points seen by this keyframe and the ones
        seen by it's neighbours in the covisibility graph. It then returns
        all the keyframes that see those points and aren't in Ki's covis graph.
        */
        unsigned int GetMapPointsAndDistantKeyframes(
            const Id<Keyframe>& Ki,
            unsigned int coVisTheta,
            thread_memory memory,
            std::vector<MapPointTrackingProxy>& mapPoints,
            std::vector<Proxy<Keyframe, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints>>& keyframes,
            std::vector<MapPointAssociation>& mapPointAssociations,
            std::vector<Id<Keyframe>>& externallyTetheredKeyframes) const;

        /*
        Updates all the map point positions and keyframe poses and
        destroys all the MapPoint outliers (Not sure if we actually destroy them or not).
        */
        void AdjustPosesAndMapPoints(
            const AdjustableData& adjusted,
            gsl::span<const std::pair<Id<MapPoint>, Id<Keyframe>>> outlierAssociations,
            thread_memory memory);

        /*
        Discards the duplicate keyframes that are
        connected to Ki in the covisibility graph.

        Returns the KeyframeIds that were removed as well as the covisability which lead to the removal decision if argument provided
        */
        void CullLocalKeyframes(
            Id<Keyframe> Ki,
            thread_memory memory,
            std::vector<std::pair<const Id<Keyframe>, const std::vector<Id<Keyframe>>>>* cullingDecisions = nullptr);

        /*
        gets keyframes connected to KId in the covisibility graph (more than the minimum required
        map points in common to be entered as a link in the covisibility graph)
        */
        void GetCovisibilityConnectedKeyframes(
            const Id<Keyframe>& kId,
            thread_memory memory,
            std::vector<MappingKeyframe>& kc) const;

        /*
        gets the list of recent points the mapping thread is still carefully evaluating. track local map
        will use this to collect more information on these points for use by the mapping thread, and the
        age of each of these map points
        */
        std::map<Id<MapPoint>, unsigned int> GetRecentlyCreatedMapPoints() const;

        /*
        * Will attempt to match the set of map points into the set of keyframes if matching criterea are met
        */
        void TryConnectMapPoints(gsl::span<const MapPointAssociations<MapPointTrackingProxy>::Association> mapPointAssociations,
            const Id<Keyframe>& insertedKeyframe,
            const TrackLocalMapSettings& trackLocalMapSettings,
            const OrbMatcherSettings& orbMatcherSettings,
            float searchRadius,
            thread_memory memory);

        /*
        Delete the state of the entire map
        */
        void Clear(thread_memory memory);

        /*
        Returns all the information pertinent when saving the map
        */
        MapState GetMapData() const;

        float GetMedianTetherDistance() const;

        float GetMapScale() const;

        /*
        Returns all the keyframes in the map
        */
        std::vector<KeyframeProxy> GetAllKeyframes() const;

        /*
        Destroys this instance of the thread safe map and returns it's
        internal map for use in another context.
        */
        static std::unique_ptr<Map> Release(std::unique_ptr<ThreadSafeMap> tsm);

    private:
        void UnSafeGetCovisibilityConnectedKeyframes(
            const Id<Keyframe>& kId,
            thread_memory memory,
            std::vector<MappingKeyframe>& kc) const;

        // the int value is the number of Keyframes this map point has existed for
        std::map<Id<MapPoint>, unsigned int> UnSafeGetRecentlyCreatedMapPoints() const;

        /*
        Performs all the logic for AdjustPosesAndMapPoints, but does not take its own
        lock and so can be used by other methods as well.
        */
        void UnSafeAdjustPosesAndMapPoints(
            const AdjustableData& adjusted,
            gsl::span<const std::pair<Id<MapPoint>, Id<Keyframe>>> outlierAssociations,
            thread_memory memory);

        // vector to record map point creations for map point culling
        historical_queue<std::vector<Proxy<MapPoint>>, 3> m_pointHistory;

        const KeyframeSettings& m_keyframeSettings;
        const MappingSettings& m_mappingSettings;
        const CovisibilitySettings& m_covisibilitySettings;
        const BundleAdjustSettings& m_bundleAdjustSettings;

        mutable std::shared_mutex m_mutex;

        std::unique_ptr<Map> m_map;

        BaseBow& m_bow;

        float m_mapScale;
    };
}
