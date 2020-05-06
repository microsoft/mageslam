// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// MapPoint.h
//
// Mappoints are 3D points that have been estimated from triangulating
// the positions of the same 2D image space feature viewed from multiple images
//------------------------------------------------------------------------------

#pragma once

#include "Image\ORBDescriptor.h"
#include "Data\Types.h"
#include "MageSettings.h"

#include <set>
#include <opencv2\core\core.hpp>

namespace mage
{
    class Keyframe;
    class Map;

    class MapPoint
    {
    public:
        MapPoint(const Id<MapPoint>& id, size_t numLevels);

        /*
            returns the unique id for the MapPoint.
        */
        const Id<MapPoint>& GetId() const
        {
            return m_id;
        }

        inline void IncrementRefinementCount()
        {
            m_positionRefinementCount++;
        }

        inline unsigned int GetRefinementCount() const
        {
            return m_positionRefinementCount;
        }

        /*
            returns the world position of the MapPoint.
        */
        void SetPosition(const cv::Point3f position);

        /*
            returns the world position of the MapPoint.
        */
        const cv::Point3f& GetPosition() const;

        /*
            returns the minimum distance at which this MapPoint
            can be seen from the keyframes that see it.
        */
        float GetDMin() const { return m_dMin; }

        /*
            returns the maximum distance at which this MapPoint
            can be seen from the keyframes that see it.
        */
        float GetDMax() const { return m_dMax; }

        /*
            Returns the average viewing direction from
            each keyframe to the current map point.
        */
        const cv::Vec3f& GetMeanViewingDirection() const
        {
            return m_meanViewingDirection;
        }

        /*
            Gets all the keyframes that can see this MapPoint.
        */
        const std::vector<Keyframe const*>& GetKeyframes() const;

        /*
            Gets the best descriptor for this MapPoint.
        */
        const ORBDescriptor& GetRepresentativeDescriptor() const;

        const size_t GetKeyPointCountAtLevel(size_t level) const;

    private:
        void UpdateRepresentativeDescriptor();
        void UpdateMeanViewDirectionAndDistances();

        /*
            These two methods are private and should only be called by the
            maps Associate and RemoveAssociation functions
        */
        void AddKeyframeAssociation(const Keyframe* keyframe, KeypointDescriptorIndex keypointIndex);
        void RemoveKeyframeAssociation(const Keyframe* keyframe, KeypointDescriptorIndex keypointIndex);
        void ClearAssociations();

        /*
            Returns the vector from the keyframes world position to
            the MapPoints world position.
        */
        cv::Vec3f CalculateDeltaWorldPositionFromKeyframe(const Keyframe* keyframe);

        cv::Point3f m_position;
        cv::Vec3f m_meanViewingDirection;

        std::vector<const Keyframe*> m_keyframes;
        const Keyframe* m_representativeDescriptorKeyframe = nullptr;
        KeypointDescriptorIndex m_representativeAssociation = (KeypointDescriptorIndex)-1;

        Id<MapPoint> m_id;

        float m_dMin = std::numeric_limits<float>::max();
        float m_dMax = std::numeric_limits<float>::min();

        // the number of times this MapPoint has had it's position refined
        unsigned int m_positionRefinementCount = 0;

        // performance optimization
        // this is a bucket count for the octaves of the keypoints for each connected map points
        // the reason for this lookup is an optimization in CullLocalKeyframes, which cuts its CPU cost by 50%
        // because it does not need to look up each keypoint in each keyframe to find the octave, but can simply
        // use these cached values
        std::vector<size_t> m_octaveCounters;

        friend void MapAssociate(Keyframe* keyframe, MapPoint* mappoint, KeypointDescriptorIndex index);
        friend KeypointDescriptorIndex MapRemoveAssociation(Keyframe* keyframe, MapPoint* mappoint);
        friend void MapRemoveAllAssociations(Map* map, MapPoint* mapPoint, std::set<const Keyframe*>& modifiedKeyframes);
    };
}

namespace std
{
    template<>
    struct hash<mage::Id<mage::MapPoint>> : private hash<mage::IdT>
    {
        size_t operator()(const mage::Id<mage::MapPoint>& id) const
        {
            return hash<mage::IdT>::operator()(id.val);
        }
    };

    template<>
    struct hash<mage::MapPoint*> : private hash<mage::Id<mage::MapPoint>>
    {
        size_t operator()(const mage::MapPoint* mp) const
        {
            return hash<mage::Id<mage::MapPoint>>::operator()(mp->GetId());
        }
    };
}
