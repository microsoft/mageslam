// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "ThreadSafeMap.h"
#include "Data\Keyframe.h"
#include "Tracking\Triangulation.h"
#include "Mapping\NewMapPointsCreation.h"
#include "BundleAdjustment\BundleAdjust.h"

#include "Data\Operators.h"

#include <set>
#include <algorithm>

#include "Utils\Logging.h"
#include "Utils\range.h"

#include "Analysis\DataFlow.h"

#include <arcana/analysis/xray.h>
#include <arcana/analysis/data_point.h>
#include <arcana/analysis/object_trace.h>

using namespace std;

namespace mage
{
    ThreadSafeMap::ThreadSafeMap(
        const MageSlamSettings& settings, BaseBow& bagOfWords)
        : m_mutex{},
        m_map{ std::make_unique<Map>(settings.CovisibilitySettings, settings.KeyframeSettings, settings.RelocalizationSettings) },
        m_keyframeSettings{ settings.KeyframeSettings },
        m_mappingSettings{ settings.MappingSettings },
        m_covisibilitySettings{ settings.CovisibilitySettings },
        m_bundleAdjustSettings{ settings.BundleAdjustSettings },
        m_mapScale{ 1.0f },
        m_bow{bagOfWords}
    {
    }

    ThreadSafeMap::~ThreadSafeMap()
    {}

    void ThreadSafeMap::InitializeMap(const InitializationData& initializationData, thread_memory memory)
    {
        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(initializationData)
        );

        SCOPE_TIMER(ThreadSafeMap::InitializeMap);
        SCOPE_TIMER(ThreadSafeMapUniqueLock);

        unique_lock<shared_mutex> lock{ m_mutex };

        assert(m_map->GetKeyframesCount() == 0 && "Map already has Keyframes");
        assert(m_map->GetMapPointsCount() == 0 && "Map already has MapPoints");

        auto keyframes = memory.stack_vector<Keyframe*>(initializationData.Frames.size());
        m_pointHistory.clear();
        std::vector<Proxy<MapPoint>> mapPointsToAdd;
        mapPointsToAdd.reserve(initializationData.MapPoints.size());

        // TODO assumes that all the initialization frames have the same numLevels
        size_t numLevels = initializationData.Frames[0]->GetAnalyzedImage()->GetNumLevels();

        // add the map points
        for (const MapPointTrackingProxy& mp : initializationData.MapPoints)
        {
            MapPoint* mapPoint = m_map->AddMapPoint(make_unique<MapPoint>(mp.GetId(), numLevels));
            mapPoint->SetPosition(mp.GetPosition());
            mapPoint->IncrementRefinementCount();
            mapPointsToAdd.push_back(mapPoint);
        }

        // add the keyframes and associate the map points
        for (const auto& builder : initializationData.Frames)
        {
            Keyframe* keyframe = m_map->AddKeyframe(make_unique<Keyframe>(*builder));
            keyframes.emplace_back(keyframe);

            builder->IterateAssociations([&](const MapPointTrackingProxy& mp, KeypointDescriptorIndex idx)
            {
                MapPoint* mapPoint = m_map->GetMapPoint(mp.GetId());
                m_map->Associate(keyframe, mapPoint, idx);
            });
        }

        keyframes.front()->SetFixed(true);
        keyframes.front()->SetImmortal(true);

        auto vecBetweenKeyframes = keyframes.front()->GetPose().GetWorldSpacePosition() - keyframes.back()->GetPose().GetWorldSpacePosition();
        float distanceBetweenKeyframes = sqrt(vecBetweenKeyframes.dot(vecBetweenKeyframes));
        keyframes.back()->AddDistanceTether(keyframes.front()->GetId(), distanceBetweenKeyframes, m_bundleAdjustSettings.DistanceTetherWeight);
        keyframes.back()->SetImmortal(true);

        m_mapScale = (float)cv::norm(keyframes.back()->GetPose().GetViewSpacePosition());

        m_pointHistory.advance(move(mapPointsToAdd));

        for (const Keyframe* kf : keyframes)
        {
            m_map->UpdateGraph(kf, memory);
        }
    }

    // returns a sampling of mappoints
    void ThreadSafeMap::GetSampleOfMapPoints(
        temp::vector<MapPointProxy>& mapPoints, uint32_t max, uint32_t offset) const
    {
        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(max, offset)
            XR_OUTPUT(mapPoints)
        );

        shared_lock<shared_mutex> lock{ m_mutex };
        m_map->GetSampleOfMapPoints(mapPoints, max, offset);
    }

    // returns connnected mappoints that are connected by keyframes to a collection of provided mappoints
    void ThreadSafeMap::GetConnectedMapPoints(
        const collection<Proxy<MapPoint>>& points,
        thread_memory memory,
        loop::vector<KeyframeReprojection>& reprojections) const
    {
        SCOPE_TIMER(ThreadSafeMap::GetConnectedMapPoints);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(points)
            XR_OUTPUT(reprojections)
        );

        SCOPE_TIMER(ThreadSafeMapSharedLock);

        shared_lock<shared_mutex> lock{ m_mutex };

        auto K1MapPointCount = memory.stack_unordered_map<mage::Id<mage::Keyframe>, uint64_t>();
        K1MapPointCount.reserve(m_map->GetKeyframesCount());

        for (const auto& point : points)
        {
            auto mp = m_map->GetMapPoint(point.GetId());
            if (mp == nullptr)
                continue;

            for (const auto& kf : mp->GetKeyframes())
            {
                K1MapPointCount[kf->GetId()]++;
            }
        }

        auto K1s = memory.stack_unique_vector<const Keyframe*, operators::keyframe::less>(K1MapPointCount.size());

        for (const auto& keyframeConnectionsCount : K1MapPointCount)
        {
            if (keyframeConnectionsCount.second >= m_covisibilitySettings.CovisMinThreshold)
            {
                K1s.insert(m_map->GetKeyframe(keyframeConnectionsCount.first));
            }
        }

        auto K2s = memory.stack_unique_vector<Id<Keyframe>>(10 * K1s.size());
        for (const auto& kf : K1s)
        {
            m_map->GetCovisibilityConnectedKeyframes(kf->GetId(), K2s);
        }

        auto K1K2s = memory.stack_unique_vector<const Keyframe*, operators::keyframe::less>(K1s.size() + K2s.size());
        for (const auto& id : K2s)
        {
            K1K2s.insert_presorted(m_map->GetKeyframe(id));
        }
        K1K2s.merge(K1s);

        reprojections.reserve(K1K2s.size());

        for (auto* keyframe : K1K2s)
        {
            auto mapPoints = memory.stack_vector<const MapPoint*>();
            keyframe->GetMapPoints(mapPoints);

            std::vector<MapPointProxy> mapPointProxies;
            mapPointProxies.reserve(mapPoints.size());

            KeyframeReprojection repro{
                keyframe->GetId(),
                keyframe->GetPose(),
                move(mapPointProxies)
            };

            for (MapPoint const* mapPoint : mapPoints)
            {
                repro.Points.emplace_back(mapPoint);
            }

            reprojections.emplace_back(move(repro));
        }
    }

    Proxy<Keyframe, proxy::Image> ThreadSafeMap::InsertKeyframe(
        const shared_ptr<const KeyframeBuilder>& kb,
        thread_memory memory)
    {
        return InsertKeyframe(kb, {}, memory);
    }

    Proxy<Keyframe, proxy::Image> ThreadSafeMap::InsertKeyframe(
        const shared_ptr<const KeyframeBuilder>& kb,
        const gsl::span<MapPointAssociations<MapPointTrackingProxy>::Association>& extraAssociation,
        thread_memory memory)
    {
        SCOPE_TIMER(ThreadSafeMap::InsertKeyframe);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(kb, extraAssociation)
        );

        SCOPE_TIMER(ThreadSafeMapUniqueLock);
        unique_lock<shared_mutex> lock{ m_mutex };

        //add keyframe to map
        auto newKf = m_map->AddKeyframe(make_unique<Keyframe>(*kb));

        // associate keyframe and map points
        vector<MapPointAssociations<MapPointTrackingProxy>::Association> mps;
        kb->GetAssociations(mps);

        for (auto& association : mps)
        {
            // map points stored on the keyframe by the tracking thread could be stale and
            // have been culled out of the map already
            auto mapPoint = m_map->GetMapPoint(association.MapPoint.GetId());
            if (mapPoint != nullptr)
            {
                m_map->Associate(newKf, mapPoint, association.Index);
            }
        }

        for (const auto& assoc : extraAssociation)
        {
            auto mapPoint = m_map->GetMapPoint(assoc.MapPoint.GetId());
            if (mapPoint != nullptr)
            {
                m_map->Associate(newKf, mapPoint, assoc.Index);
            }
        }

        FIRE_OBJECT_TRACE("Keyframes.Total", this, (mira::make_data_point<float>(
            newKf->GetAnalyzedImage()->GetTimeStamp(),
            (float)m_map->GetKeyframesCount())));

        m_map->UpdateGraph(newKf, memory);
        assert(m_map->ValidSpanningTree());

        return newKf;
    }

    float ThreadSafeMap::GetMedianTetherDistance() const
    {
        std::vector<float> distancesSquare;

        {
            shared_lock<shared_mutex> lock{ m_mutex }; //TODO: We don't want external clients to be able to grab a lock on our map

            m_map->IterateKeyframes([&](const Keyframe& kf)
            {
                const auto& tethers = kf.GetTethers();

                for (const auto& tether : tethers)
                {
                    if (tether.Type() != TetherType::EXTRINSIC)
                        continue;

                    const auto* ko = m_map->GetKeyframe(tether.OriginId());
                    cv::Vec3f deltaPosition = kf.GetPose().GetWorldSpacePosition() - ko->GetPose().GetWorldSpacePosition();
                    distancesSquare.push_back(deltaPosition.dot(deltaPosition));
                }
            });
        }

        if (distancesSquare.empty())
        {
            return 0;
        }

        return  mira::median<float>(distancesSquare.begin(), distancesSquare.end(), std::sqrtf);
    }

    float ThreadSafeMap::GetMapScale() const
    {
        return m_mapScale;
    }

    std::vector<KeyframeProxy> ThreadSafeMap::GetAllKeyframes() const
    {
        shared_lock<shared_mutex> lock{ m_mutex };

        std::vector<KeyframeProxy> proxies;

        auto ids = m_map->GetKeyframeIds();

        proxies.reserve(ids.size());
        std::transform(ids.begin(), ids.end(), mira::emplace_inserter(proxies),
            [this](const Id<Keyframe>& id)
            {
                return m_map->GetKeyframe(id);
            });

        return proxies;
    }

    void ThreadSafeMap::UpdateKeyframesFromProxies(
        gsl::span<const KeyframeProxy> proxies,
        const OrbMatcherSettings& mergeSettings,
        std::unordered_map<Id<MapPoint>, MapPointTrackingProxy>& mapPointMerges,
        thread_memory memory)
    {
        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(proxies)
        );

        SCOPE_TIMER(ThreadSafeMap::UpdateKeyframesFromProxies);
        unique_lock<shared_mutex> lock{ m_mutex };

        m_map->UpdateKeyframesFromProxies(proxies, mergeSettings, mapPointMerges);

        m_map->IterateKeyframes([&](const Keyframe& kf)
        {
            m_map->UpdateGraph(&kf, memory);
        });

        m_map->LogSnapshot();
    }

    void ThreadSafeMap::UpdateMapPoints(KeyframeProxy& proxy) const
    {
        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(proxy)
            XR_OUTPUT(proxy)
        );

        SCOPE_TIMER(ThreadSafeMap::UpdateKeyframesFromProxies);
        unique_lock<shared_mutex> lock{ m_mutex };

        m_map->UpdateMapPoints(proxy);
    }

    void ThreadSafeMap::BuildGlobalBundleAdjustData(AdjustableData& data) const
    {
        SCOPE_TIMER(ThreadSafeMap::BuildGlobalBundleAdjustData);
        unique_lock<shared_mutex> lock{ m_mutex };

        m_map->IterateMapPoints([&](const MapPoint& mp)
        {
            data.MapPoints.emplace_back(mp);
        });

        m_map->IterateKeyframes([&](const Keyframe& kf)
        {
            data.Keyframes.emplace_back(kf);

            // Fix all distance-tethered keyframes for the global bundle adjust step as rotational
            // variances can have a catastrophic impact when bundle adjusting the whole world at the same
            // time.
            // TODO: Arbitrarily fixing the positions of distance-tethered keyframes is kludge
            for (const auto& tether : kf.GetTethers())
            {
                if (tether.Type() == TetherType::DISTANCE)
                {
                    data.Keyframes.back().SetFixed(true);
                    break;
                }
            }

            kf.IterateAssociations([&](const MapPoint* mp, KeypointDescriptorIndex idx)
            {
                MapPointAssociation assoc{
                    kf.GetAnalyzedImage()->GetKeyPoint(idx).pt,
                    mp->GetId(),
                    kf.GetId()
                };
                data.MapPointAssociations.emplace_back(assoc);
            });
        });
    }

    void ThreadSafeMap::FindSimilarKeyframes(const shared_ptr<const AnalyzedImage>& image, vector<KeyframeProxy>& output) const
    {
        SCOPE_TIMER(ThreadSafeMap::FindSimilarKeyframes);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(image)
            XR_OUTPUT(output)
        );

        auto res = m_bow.QueryUnknownImage(image->GetDescriptors(), m_mappingSettings.MaxRelocQueryResults);
        {
            SCOPE_TIMER(ThreadSafeMapUniqueLock);

            unique_lock<shared_mutex> lock{ m_mutex };
            for (Id<Keyframe> id : res)
            {
                auto framePtr = m_map->GetKeyframe(id);
                if (framePtr != nullptr)
                {
                    output.emplace_back(framePtr);
                }
                else
                {
                    LogMessage<Tracing::TraceLevels::Warning>(L"QueryUnknownImage returned an element that was not in the map.  This could be a timing quirk or a map/BoW desync bug.");
                }
            }
        }
    }

    void ThreadSafeMap::FindNonCovisibleSimilarKeyframeClusters(const KeyframeProxy& proxy, thread_memory memory, std::vector<std::vector<KeyframeProxy>>& output) const
    {
        SCOPE_TIMER(ThreadSafeMap::FindNonCovisibleSimilarKeyframeClusters);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(proxy)
            XR_OUTPUT(output)
        );

        auto queryResults = m_bow.QueryUnknownImage(proxy.GetAnalyzedImage()->GetDescriptors(), m_mappingSettings.MaxLoopClosureQueryResults);
        auto idsToScores = memory.stack_unordered_map<Id<Keyframe>, float>();
        std::transform(queryResults.begin(), queryResults.end(), std::inserter(idsToScores, idsToScores.end()),
            [](const BaseBow::QueryMatch& match) { return std::pair<Id<Keyframe>, float>{ match.Id, match.Score }; });

        // Remove yourself; no sense in loop closing to the place you already are.
        idsToScores.erase(proxy.GetId());

        float lowestCovisScore = std::numeric_limits<float>::max();

        // Begin section that uses the map directly.
        {
            unique_lock<shared_mutex> lock{ m_mutex };

            // Find the lowest covisibility score and remove all locally covisible keyframe IDs.
            // TODO: Reserve a reasonable size when one can be determined (setting?).
            auto localCovis = memory.stack_unique_vector<Id<Keyframe>>();
            m_map->GetCovisibilityConnectedKeyframes(proxy.GetId(), localCovis, m_covisibilitySettings.CovisLoopThreshold);
            for (const auto& id : localCovis)
            {
                auto it = idsToScores.find(id);
                if (it != idsToScores.end())
                {
                    lowestCovisScore = min(lowestCovisScore, it->second);
                    idsToScores.erase(it);
                }
            }

            // Remove all IDs that score too low.
            auto goodIds = memory.loop_set<Id<Keyframe>>();
            for (const auto& pair : idsToScores)
            {
                if (pair.second >= lowestCovisScore)
                {
                    goodIds.insert(pair.first);
                }
            }

            auto clusters = std::vector<std::vector<Id<Keyframe>>>();
            m_map->GetCovisibilityConnectedSubGraphs(goodIds, memory, clusters);

            // Convert clusters of IDs into clusters of keyframe proxies for returning.
            for (const auto& cluster : clusters)
            {
                output.emplace_back();

                for (const auto& id : cluster)
                {
                    auto kf = m_map->GetKeyframe(id);
                    if (kf != nullptr)
                    {
                        output.back().emplace_back(kf);
                    }
                }
            }
        }
    }
    
    void ThreadSafeMap::CullRecentMapPoints(
        const Id<Keyframe>& ki,
        const mira::unique_vector<Id<MapPoint>>& recentFailedMapPoints,
        thread_memory memory)
    {
        SCOPE_TIMER(ThreadSafeMap::CullRecentMapPoints);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(ki, recentFailedMapPoints)
        );

        SCOPE_TIMER(ThreadSafeMapUniqueLock);

        unique_lock<shared_mutex> lock{ m_mutex };

        set<MapPoint*> mapPointsToCull;
        for (size_t idxPointSet = 0; idxPointSet < m_pointHistory.size(); idxPointSet++)
        {
            vector<Proxy<MapPoint>>& points = m_pointHistory[idxPointSet];
            points.erase(std::remove_if(points.begin(), points.end(), [&](const Proxy<MapPoint>& mapPointProxy)
            {
                MapPoint* mapPoint = m_map->GetMapPoint(mapPointProxy.GetId());
                if (mapPoint == nullptr)
                {
                    // mapPoint cleaned up by local keyframe culling or bundle adjust outliers
                    return true;
                }
                else if ((idxPointSet > 0) && (mapPoint->GetKeyframes().size() < m_mappingSettings.MinNumKeyframesForMapPointCulling))
                {
                    //  if mappoint is created in keyframe insertion n, the map point will remain in slot 0 of point history
                    //  for map points culling for keyframe insertion n+1 (the history is advanced on create map points). the paper seems
                    //  to state that we need to wait for an additional keyframe insertion before enforcing this test. this is why we only consider
                    //  points in slot 1 or higher, making this test apply only in keyframe insertion n+2 and n+3

                    // map point visible in less than 3 keyframes should be removed
                    mapPointsToCull.insert(mapPoint);
                    return true;
                }
                else if (recentFailedMapPoints.end() != recentFailedMapPoints.find(mapPoint->GetId()))
                {
                    //The tracking must find the point in more than the 25 % of the frames in which it is predicted to be visible.
                    mapPointsToCull.insert(mapPoint);
                    return true;
                }

                return false;
            }), points.end());
        }

        unsigned int numMapPointsCulled = 0;

        set<const Keyframe*> modified;
        for (MapPoint* mapPoint : mapPointsToCull)
        {
            auto& kfs = mapPoint->GetKeyframes();
            auto foundPoint = std::find_if(kfs.begin(), kfs.end(), 
                [&ki](const Keyframe* kf)
            {
                return kf->GetId() == ki;
            });

            // only remove mappoint if not seen in current keyframe
            if (foundPoint == kfs.end() || m_pointHistory.size() == 1)
            {
                m_map->RemoveAllAssociations(mapPoint, modified);
                m_map->RemoveMapPoint(mapPoint->GetId());
                numMapPointsCulled++;
            }
            else
            {
                if (m_pointHistory.full())
                {
                    // oldest after next step.
                    (m_pointHistory.rbegin() + 1)->emplace_back(mapPoint);
                }
                else
                {
                    m_pointHistory.oldest().emplace_back(mapPoint);
                }
            }
        }

        FIRE_OBJECT_TRACE("Mappoints.Culled", this, (mira::make_data_point<float>(
            m_map->GetKeyframe(ki)->GetAnalyzedImage()->GetTimeStamp(),
            (float)numMapPointsCulled)));

        FIRE_OBJECT_TRACE("Mappoints.Total", this, (mira::make_data_point<float>(
            m_map->GetKeyframe(ki)->GetAnalyzedImage()->GetTimeStamp(),
            (float)m_map->GetMapPointsCount())));

        for (const Keyframe* kf : modified)
        {
            m_map->UpdateGraph(kf, memory);
        }
        assert(m_map->ValidSpanningTree());
    }

    void ThreadSafeMap::GetMapPointsAsPositions(std::vector<Position>& positions) const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        m_map->GetMapPointsAsPositions(positions);
    }

    size_t ThreadSafeMap::GetMapPointsCount() const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        return m_map->GetMapPointsCount();
    }

    size_t ThreadSafeMap::GetKeyframesCount() const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        return m_map->GetKeyframesCount();
    }

    void ThreadSafeMap::GetKeyframeViewMatrices(std::vector<Matrix>& viewMatrices) const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        m_map->GetKeyframeViewMatrices(viewMatrices);
    }

    vector<MappingKeyframe> ThreadSafeMap::CreateMapPoints(
        const Id<Keyframe>& keyframeId,
        const gsl::span<const MapPointKeyframeAssociations>& newMapPoints,
        thread_memory memory)
    {
        SCOPE_TIMER(ThreadSafeMap::CreateMapPoints);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(keyframeId, newMapPoints)
        );

        vector<Proxy<MapPoint>> converted;
        converted.reserve(newMapPoints.size());

        SCOPE_TIMER(ThreadSafeMapUniqueLock);

        unique_lock<shared_mutex> lock{ m_mutex };

        set<Keyframe*> modified;

        auto pKeyframe = m_map->GetKeyframe(keyframeId);
        const size_t numLevels = pKeyframe->GetAnalyzedImage()->GetNumLevels();

        // add map points to the map
        for (const auto& mpProxy : newMapPoints)
        {
            converted.emplace_back(mpProxy.MapPoint.As<Proxy<MapPoint>>());

            MapPoint* mapPoint = m_map->AddMapPoint(make_unique<MapPoint>(mpProxy.MapPoint.GetId(), numLevels));
            mapPoint->SetPosition(mpProxy.MapPoint.GetPosition());

            // associate each map point
            for (const KeyframeAssociation& curKeyframe : mpProxy.Keyframes)
            {
                Keyframe* keyframe = m_map->GetKeyframe(curKeyframe.KeyframeId);
                m_map->Associate(keyframe, mapPoint, curKeyframe.KeypointDescriptorIndex);

                modified.insert(keyframe);
            }
        }

        FIRE_OBJECT_TRACE("Mappoints.Created", this, (mira::make_data_point<float>(
            pKeyframe->GetAnalyzedImage()->GetTimeStamp(),
            (float)newMapPoints.size())));

        FIRE_OBJECT_TRACE("Mappoints.Total", this, (mira::make_data_point<float>(
            pKeyframe->GetAnalyzedImage()->GetTimeStamp(),
            (float)m_map->GetMapPointsCount())));

        //increment history
        m_pointHistory.advance(move(converted));

        // update the covisibility graph for any keyframes we have created new connections to
        for (auto* kf : modified)
        {
            m_map->UpdateGraph(kf, memory);
        }
        assert(m_map->ValidSpanningTree());

        //get copy of connected keyframes and store in mapping frames for use by mapping thread
        vector<MappingKeyframe> connectedKeyframes;
        UnSafeGetCovisibilityConnectedKeyframes(keyframeId, memory, connectedKeyframes);
        return connectedKeyframes;
    }

    unique_ptr<KeyframeProxy> ThreadSafeMap::GetKeyFrameProxy(const Id<Keyframe>& id) const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        Keyframe *ptr = m_map->GetKeyframe(id);
        return ptr == nullptr ? nullptr : make_unique<KeyframeProxy>(ptr);
    }

    void ThreadSafeMap::GetCovisibilityConnectedKeyframes(
        const Id<Keyframe>& kId,
        thread_memory memory,
        std::vector<MappingKeyframe>& kcs) const
    {
        SCOPE_TIMER(ThreadSafeMap::GetCovisibilityConnectedKeyframes);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(kId)
            XR_OUTPUT(kcs)
        );

        SCOPE_TIMER(ThreadSafeMapSharedLock);

        shared_lock<shared_mutex> lock{ m_mutex };

        UnSafeGetCovisibilityConnectedKeyframes(kId, memory, kcs);
    }

    std::map<Id<MapPoint>, unsigned int> ThreadSafeMap::GetRecentlyCreatedMapPoints() const
    {
        SCOPE_TIMER(ThreadSafeMap::GetRecentlyCreatedMapPointsList);
        SCOPE_TIMER(ThreadSafeMapSharedLock);

        shared_lock<shared_mutex> lock{ m_mutex };

        return UnSafeGetRecentlyCreatedMapPoints();
    }

    void ThreadSafeMap::TryConnectMapPoints(gsl::span<const MapPointAssociations<MapPointTrackingProxy>::Association> mapPointAssociations,
        const Id<Keyframe>& insertedKeyframe,
        const TrackLocalMapSettings& trackLocalMapSettings,
        const OrbMatcherSettings& orbMatcherSettings,
        float searchRadius,
        thread_memory memory)
    {
        // Propagate newly connected map points to the connected keyframes
        SCOPE_TIMER(ThreadSafeMapUniqueLock);
        unique_lock<shared_mutex> lock{ m_mutex };

        SCOPE_TIMER(ThreadSafeMap::TryConnectMapPoints);

        auto connectedKfs = memory.stack_unique_vector<Id<Keyframe>>(100);
        m_map->GetCovisibilityConnectedKeyframes(insertedKeyframe, connectedKfs);

        temp::vector<Keyframe*> modifiedKeyframes = memory.stack_vector<Keyframe*>(connectedKfs.size());

        for (const Id<Keyframe>& keyframeId : connectedKfs)
        {
            vector<MapPointAssociations<MapPointTrackingProxy>::Association> extraAssociations;
            Keyframe* pKeyframe = m_map->GetKeyframe(keyframeId);

            const cv::Matx33f& cameraCalibrationMatrix = pKeyframe->GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix();
            const Pose& pose = pKeyframe->GetPose();
            const cv::Matx34f& viewMatrix = pose.GetViewMatrix();
            const cv::Point3f framePosition = pose.GetWorldSpacePosition();
            const cv::Vec3f frameForward = pose.GetWorldSpaceForward();
            vector<bool> unassociatedMask = pKeyframe->CreateUnassociatedMask();

            const float matchRadius = pKeyframe->GetAnalyzedImage()->GetImageBorder() - searchRadius / 2.0f;
            const float pyramidScale = pKeyframe->GetAnalyzedImage()->GetPyramidScale();
            const size_t numLevels = pKeyframe->GetAnalyzedImage()->GetNumLevels();

            for (const auto& mapPointAssociation : mapPointAssociations)
            {
                // check to ensure that the map point isn't already associated with the keyframe
                if (!pKeyframe->IsAssociatedToMapPoint(mapPointAssociation.MapPoint.GetId()))
                {
                    // try to match the map point into the projection of the keyframe
                    bool predicted = false;
                    KeypointDescriptorIndex keypointDescriptorIndex;
                    MapPoint* mapPoint = m_map->GetMapPoint(mapPointAssociation.MapPoint.GetId());

                    if (TrackLocalMap::ProjectMapPointIntoCurrentFrame(*pKeyframe, viewMatrix, cameraCalibrationMatrix, *mapPoint, framePosition,
                        frameForward, trackLocalMapSettings.MinDegreesBetweenCurrentViewAndMapPointView, searchRadius,
                        matchRadius, pyramidScale, numLevels,
                        orbMatcherSettings, unassociatedMask, memory, predicted, keypointDescriptorIndex))
                    {
                        extraAssociations.emplace_back(*mapPoint, keypointDescriptorIndex);
                        unassociatedMask[keypointDescriptorIndex] = false;
                    }
                }
            }

            // apply any found associations to the keyframe
            for (const auto& assoc : extraAssociations)
            {
                m_map->Associate(pKeyframe, m_map->GetMapPoint(assoc.MapPoint.GetId()), assoc.Index);
            }

            if (extraAssociations.size() > 0)
            {
                modifiedKeyframes.push_back(pKeyframe);
            }
        }

        // update the covis and spanning trees to account for the new associations
        for (Keyframe* pKeyframe : modifiedKeyframes)
        {
            m_map->UpdateGraph(pKeyframe, memory);
        }
    }

    void ThreadSafeMap::Clear(thread_memory memory)
    {
        m_bow.Clear();

        {
            SCOPE_TIMER(ThreadSafeMapUniqueLock);

            unique_lock<shared_mutex> lock{ m_mutex };

            m_pointHistory.clear();

            m_map->Clear(memory);
        }
    }

    MapState ThreadSafeMap::GetMapData() const
    {
        MapState state;

        shared_lock<shared_mutex> lock{ m_mutex };

        state.MapPoints.reserve(m_map->GetMapPointsCount());
        m_map->IterateMapPoints([&](const MapPoint& mp)
        {
            state.MapPoints.emplace_back(mp);
        });

        state.Keyframes.reserve(m_map->GetKeyframesCount());
        m_map->IterateKeyframes([&](const Keyframe& kf)
        {
            state.Keyframes.emplace_back(kf);
        });

        return state;
    }

    std::unique_ptr<Map> ThreadSafeMap::Release(std::unique_ptr<ThreadSafeMap> tsm)
    {
        return std::move(tsm->m_map);
    }

    void ThreadSafeMap::UnSafeGetCovisibilityConnectedKeyframes(
        const Id<Keyframe>& kId,
        thread_memory memory,
        std::vector<MappingKeyframe>& kcs) const
    {
        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(kId)
            XR_OUTPUT(kcs)
        );

        SCOPE_TIMER(ThreadSafeMap::UnSafeGetCovisibilityConnectedKeyframes);

        auto connectedKfs = memory.stack_unique_vector<Id<Keyframe>>(100);
        m_map->GetCovisibilityConnectedKeyframes(kId, connectedKfs);

        kcs.reserve(connectedKfs.size());
        for (const auto& id : connectedKfs)
        {
            kcs.emplace_back(m_map->GetKeyframe(id));
        }
    }

    std::map<Id<MapPoint>, unsigned int> ThreadSafeMap::UnSafeGetRecentlyCreatedMapPoints() const
    {
        std::map<Id<MapPoint>, unsigned int> recentPoints;
        for (unsigned int age = 0; age < m_pointHistory.size(); ++age)
        {
            const std::vector<Proxy<MapPoint>>& curMapPoints = m_pointHistory[age];
            for (const Proxy<MapPoint>& mapPoint : curMapPoints)
            {
                recentPoints.emplace(mapPoint.GetId(), age);
            }
        }

        return recentPoints;
    }

    unsigned int ThreadSafeMap::GetMapPointsAndDistantKeyframes(
        const Id<Keyframe>& Ki,
        unsigned int coVisTheta,
        thread_memory memory,
        std::vector<MapPointTrackingProxy>& mapPoints,
        std::vector<Proxy<Keyframe, proxy::Pose, proxy::Intrinsics, proxy::PoseConstraints>>& keyframes,
        std::vector<MapPointAssociation>& mapPointAssociations,
        std::vector<Id<Keyframe>>& externallyTetheredKeyframes) const
    {
        SCOPE_TIMER(ThreadSafeMap::GetMapPointsAndDistantKeyframes);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(Ki, coVisTheta)
            XR_OUTPUT(mapPoints, keyframes, mapPointAssociations, externallyTetheredKeyframes)
        );

        SCOPE_TIMER(ThreadSafeMapSharedLock);

        shared_lock<shared_mutex> lock{ m_mutex };

        //Plus 1 on max step so that it runs at least once to get the values.
        for (unsigned int i=0; i< m_covisibilitySettings.MaxSteps+1; i++)
        {
            mapPoints.clear();
            keyframes.clear();
            mapPointAssociations.clear();

            // Get all connected keyframes from covis graph
            auto Kc = memory.stack_unique_vector<Id<Keyframe>>();
            m_map->GetCovisibilityConnectedKeyframes(Ki, Kc, coVisTheta);
            Kc.insert(Ki);

            auto mapPointSet = memory.stack_unordered_set<const MapPoint*, std::hash<MapPoint*>>();
            mapPointSet.reserve(gsl::narrow_cast<size_t>(ceil(Kc.size() * m_map->GetKeyframe(Ki)->GetAnalyzedImage()->GetMaxFeatures())));

            // Get all map points in any of the connected key frames
            for (const Id<Keyframe>& keyframe : Kc)
            {
                auto pointsTmpVector = memory.stack_vector<const MapPoint*>();
                m_map->GetKeyframe(keyframe)->GetMapPoints(pointsTmpVector);

                for (const MapPoint* mapPoint : pointsTmpVector)
                {
                    mapPointSet.insert(mapPoint);
                }
            }

            auto keyframeSet = memory.stack_unique_vector<const Keyframe*, operators::keyframe::less>(m_map->GetKeyframesCount());

            // store map points and get all key frames that see those map points
            mapPoints.reserve(mapPointSet.size());
            for (const MapPoint* mapPoint : mapPointSet)
            {
                mapPoints.emplace_back(mapPoint);

                for (const Keyframe *keyframe : mapPoint->GetKeyframes())
                {
                    keyframeSet.insert(keyframe);

                    const cv::Point2f &p2f = keyframe->GetAssociatedKeyPoint(mapPoint).pt;

                    mapPointAssociations.emplace_back(MapPointAssociation{ p2f, mapPoint->GetId(), keyframe->GetId() });
                }
            }

            // store keyframes and get/store mapPoint 2 keyframe connections.
            keyframes.reserve(keyframeSet.size());
            for (const Keyframe* keyframe : keyframeSet)
            {
                keyframes.emplace_back(keyframe);
                keyframes.back().SetFixed(keyframe->IsFixed() || Kc.find(keyframe->GetId()) == Kc.end());
            }

            // Too many connections for BA: increase covis to reduce number of connections
            if (mapPointAssociations.size() > m_covisibilitySettings.UpperConnectionsForBA)
            {
                coVisTheta += m_covisibilitySettings.CovisBaStepThreshold;
                continue;
            }

            // Too little connections for BA to get good tracking: decrease covis to increase number of connections
            if (mapPointAssociations.size() < m_covisibilitySettings.LowerConnectionsForBA && coVisTheta > m_covisibilitySettings.CovisMinThreshold)
            {
                coVisTheta -= m_covisibilitySettings.CovisBaStepThreshold;
                continue;
            }

            break;
        }

        {
            externallyTetheredKeyframes.clear();

            set<Id<Keyframe>> queryIds;
            transform(keyframes.begin(), keyframes.end(), inserter(queryIds, queryIds.end()), [](const auto& kf) { return kf.GetId(); });

            auto tetheredIds = memory.stack_unique_vector<Id<Keyframe>>();
            m_map->FindExternallyTetheredKeyframesInSet(queryIds, tetheredIds);
            transform(tetheredIds.begin(), tetheredIds.end(), back_inserter(externallyTetheredKeyframes), [](const Id<Keyframe>& id) {return id; });
        }

        return coVisTheta;
    }

    void ThreadSafeMap::AdjustPosesAndMapPoints(
        const AdjustableData& adjusted,
        gsl::span<const std::pair<Id<MapPoint>, Id<Keyframe>>> outlierAssociations,
        thread_memory memory)
    {
        SCOPE_TIMER(ThreadSafeMap::AdjustPosesAndMapPoints);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(adjusted)
            XR_OUTPUT(outlierAssociations)
        );

        SCOPE_TIMER(ThreadSafeMapUniqueLock);

        unique_lock<shared_mutex> lock{ m_mutex };

        UnSafeAdjustPosesAndMapPoints(adjusted, outlierAssociations, memory);

        m_map->LogSnapshot();
    }

    void ThreadSafeMap::UnSafeAdjustPosesAndMapPoints(
        const AdjustableData& adjusted,
        gsl::span<const pair<Id<MapPoint>, Id<Keyframe>>> outlierAssociations,
        thread_memory memory)
    {
        std::set<const Keyframe*> modifiedKeyframes;
        for (const auto& outlierAssociation : outlierAssociations)
        {
            auto keyframe = m_map->GetKeyframe(outlierAssociation.second);
            auto mapPoint = m_map->GetMapPoint(outlierAssociation.first);
            if (keyframe != nullptr && mapPoint != nullptr)
            {
                m_map->RemoveAssociation(keyframe, mapPoint);

                if (mapPoint->GetKeyframes().size() < m_mappingSettings.MinNumKeyframesForMapPointCulling)
                {
                    m_map->RemoveAllAssociations(mapPoint, modifiedKeyframes);
                    m_map->RemoveMapPoint(mapPoint->GetId());
                }
            }
        }

        for (const auto& mappingKeyframe : adjusted.Keyframes)
        {
            // skip keyframes that were marked as fixed
            if (mappingKeyframe.IsFixed())
                continue;

            auto keyframe = m_map->GetKeyframe(mappingKeyframe.GetId());
            if (keyframe != nullptr)
            {
                keyframe->SetPose(mappingKeyframe.GetPose());
            }
        }

        // Update Map points after Keyframes so they correctly update mean viewing angle and distance for the map points.
        for (const auto& mapPointProxy : adjusted.MapPoints)
        {
            auto mapPoint = m_map->GetMapPoint(mapPointProxy.GetId());
            if (mapPoint != nullptr)
            {
                mapPoint->SetPosition(mapPointProxy.GetPosition());
                mapPoint->IncrementRefinementCount();
            }
        }

        for (const Keyframe* modKeyframe : modifiedKeyframes)
        {
            m_map->UpdateGraph(modKeyframe, memory);
        }
        assert(m_map->ValidSpanningTree());
    }

    // implementation according to section VI.E of the paper "Local Keyframe Culling"
    void ThreadSafeMap::CullLocalKeyframes(
        Id<Keyframe> Ki_ID,
        thread_memory memory,
        std::vector<std::pair<const Id<Keyframe>, const std::vector<Id<Keyframe>>>>* cullingDecisions)
    {
        SCOPE_TIMER(ThreadSafeMap::CullLocalKeyframes);

        XRAY_FUNCTION(
            "ThreadSafeMap",
            XR_INPUT(Ki_ID)
            XR_OUTPUT(cullingDecisions)
        );

        auto keyFramesToRemove = memory.stack_vector<Id<Keyframe>>();
        {
            SCOPE_TIMER(ThreadSafeMapUniqueLock);

            unique_lock<shared_mutex> lock{ m_mutex };

            auto connectedKeyFrames = memory.stack_unique_vector<Id<Keyframe>>();
            m_map->GetCovisibilityConnectedKeyframes(Ki_ID, connectedKeyFrames);

            auto sortedKeyframes = memory.stack_sorted_vector<Id<Keyframe>, operators::keyframe::less>(connectedKeyFrames.size());
            sortedKeyframes.insert(connectedKeyFrames.begin(), connectedKeyFrames.end());

            CameraIdentity camId = m_map->GetKeyframe(Ki_ID)->GetAnalyzedImage()->GetCameraIdentity();
            // iterate over the keyframes
            // keyframes are reversed so that the newest keyframe is culled first to stop a cascading effect. this distributes keyframes better instead of culling leaving large areas with no key frames.
            for (const Id<Keyframe>& KcId : reversed(sortedKeyframes))
            {
                if (KcId == Ki_ID)
                {
                    continue;
                }

                auto Kc = m_map->GetKeyframe(KcId);

                // only take into account keyframes from the same camera when checking for candidates to cull
                if (camId != Kc->GetAnalyzedImage()->GetCameraIdentity())
                {
                    continue;
                }

                // Immortal keyframes are special and help to keep the system origin and scale stable
                // For this reason, we don't want to cull them keyframes.
                if (Kc->IsImmortal())
                {
                    continue;
                }

                vector<const MapPoint*> mapPoints;
                Kc->GetMapPoints(mapPoints);

                // count how many of our map points are seen at equal or finer scale in other keyframes,
                // then if 90% of our map points are better observed in other keyframes, cull this one
                // as it doesn't bring any new information
                // this is the same as computing that at least 10% of the map points are NOT observed better, which for the case of "keep this keyframe" should be faster to compute
                int keepPointThreshold = gsl::narrow_cast<int>(mapPoints.size()) - (int)(m_keyframeSettings.MappingMaxTrackingPointOverlap * mapPoints.size());
                int keepPointCount = 0;
                for(const MapPoint* mapPoint : mapPoints)
                {
                    const cv::KeyPoint& localKeyPoint = Kc->GetAssociatedKeyPoint(mapPoint);

                    size_t numberOfKeyframesWhereThisMapPointIsSeenEqualOrFinerScale = 0;
                    for (size_t octave = 0; octave <= static_cast<size_t>(localKeyPoint.octave); octave++)
                    {
                        numberOfKeyframesWhereThisMapPointIsSeenEqualOrFinerScale += mapPoint->GetKeyPointCountAtLevel(octave);
                    }

                    // we know that the numberOfKeyframesWhereThisMapPointIsSeenEqualOrFinerScale includes Kc, so we need to subtract 1 from it's value
                    if ((numberOfKeyframesWhereThisMapPointIsSeenEqualOrFinerScale - 1) < m_keyframeSettings.MinimumKeyframeCovisibilityCount)
                    {
                        keepPointCount++;
                        // don't need to keep checking, we have already found enough points to pass the threshold
                        if (keepPointCount >= keepPointThreshold)
                        {
                            break;
                        }
                    }
                }

                // if most of of our map points are seen better in other keyframes,
                // then we have no need to exist, so cull us.
                if (keepPointCount < keepPointThreshold)
                {
                    // need to gather the covisiblity frames for this keyframe so that we can update the history
                    if (cullingDecisions != nullptr)
                    {
                        auto K2s = memory.stack_unique_vector<Id<Keyframe>>();
                        m_map->GetCovisibilityConnectedKeyframes(KcId, K2s);

                        // the K2s are a stack vector, but we need to scope to persist, so I have to make a copy of the data
                        // TODO This is so very sad, the alternative would be to have some ultra fancy templating
                        // so that the Map just fills in the right container with the appropriate allocators
                        // this could be handled differently if a looped_scoped memory was provided which could then
                        // be passed into GetCovisibilityConnectedKeyframes.  For now, just use the heap, we can revisit when necessary
                        std::vector<Id<Keyframe>> covisCopies(K2s.begin(), K2s.end());
                        cullingDecisions->push_back({ KcId, covisCopies });
                    }

                    m_map->RemoveAllAssociations(const_cast<Keyframe*>(Kc));
                    m_map->RemoveKeyframe(const_cast<Keyframe*>(Kc), memory);

                    keyFramesToRemove.push_back(KcId);

                    // if a map point is viewed from fewer than 3 keyframes after a keyframe
                    // removal the map point should be removed
                    set<const Keyframe*> modifiedKeyframes;
                    for (const MapPoint* mapPoint : mapPoints)
                    {
                        if (mapPoint->GetKeyframes().size() < 2)
                        {
                            m_map->RemoveAllAssociations(const_cast<MapPoint*>(mapPoint), modifiedKeyframes);
                            m_map->RemoveMapPoint(mapPoint->GetId());
                        }
                        else if (mapPoint->GetKeyframes().size() < m_mappingSettings.MinNumKeyframesForMapPointCulling)
                        {
                            m_pointHistory.oldest().emplace_back(mapPoint);
                        }
                    }

                    for (const Keyframe* modKeyframe : modifiedKeyframes)
                    {
                        m_map->UpdateGraph(modKeyframe, memory);
                    }
                    assert(m_map->ValidSpanningTree());
                }
            }

            FIRE_OBJECT_TRACE("Keyframes.Culled", this, (mira::make_data_point<float>(
                m_map->GetKeyframe(Ki_ID)->GetAnalyzedImage()->GetTimeStamp(),
                (float)keyFramesToRemove.size())));

            FIRE_OBJECT_TRACE("Keyframes.Total", this, (mira::make_data_point<float>(
                m_map->GetKeyframe(Ki_ID)->GetAnalyzedImage()->GetTimeStamp(),
                (float)m_map->GetKeyframesCount())));
        }
       
        {
            SCOPE_TIMER(ThreadSafeMap::BowImageRemoval);

            for (const auto& id : keyFramesToRemove)
            {
                m_bow.RemoveImage(id);
            }
        }
    }
}
