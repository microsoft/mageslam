// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Image/AnalyzedImage.h"
#include "Containers/MapPointAssociations.h"
#include "Pose.h"
#include "Intrinsics.h"
#include "Image/ORBDescriptor.h"
#include "Data/Types.h"
#include "Data/Tether.h"

#include <set>

namespace mage
{
    class KeyframeBuilder;
    class MapPoint;
    class Map;

    class Keyframe
    {
    public:
        static const float DISABLED_TETHER_DISTANCE;

        Keyframe(const KeyframeBuilder& kf_builder);

        const Id<Keyframe> GetId() const
        {
            return m_id;
        }

        //pose information
        const Pose& GetPose() const
        {
            return m_pose;
        }

        // Set Pose only called by LocalBundle adjustment so no need to update map point viewing angle or distance.
        void SetPose(const Pose &pose)
        {
            assert(!IsFixed());

            // copy the pose values and store them internally
            m_pose = pose;
            // Set Pose only called by LocalBundle adjusment so no need to update map point viewing angle or distance.
            // TODO: add update if ever need to set pose from else where.
        }

        const Intrinsics& GetUndistortedIntrinsics() const
        {
            return m_undistortedIntrinsics;
        }

        void SetUndistortedIntrinsics(const Intrinsics &intrinsics)
        {
            assert(!IsFixed());

            m_undistortedIntrinsics = intrinsics;
        }

        //descriptors 
        bool HasAssociatedMapPoint(const Id<MapPoint>& id) const
        {
            return m_associations.HasMapPoint(id);
        }

        const ORBDescriptor& GetAssociatedDescriptor(const MapPoint* mappoint) const
        {
            return m_associations.GetAssociatedDescriptor(mappoint);
        }

        const cv::KeyPoint& GetAssociatedKeyPoint(const MapPoint* mappoint) const
        {
            return m_associations.GetAssociatedKeyPoint(mappoint);
        }

        KeypointDescriptorIndex GetAssociatedIndex(const MapPoint* mappoint) const
        {
            return m_associations.GetAssociatedIndex(mappoint);
        }

        const MapPoint* GetAssociatedMapPoint(KeypointDescriptorIndex index) const
        {
            auto ptrPtr = m_associations.GetAssociatedMapPoint(index);
            if (ptrPtr == nullptr)
            {
                return nullptr;
            }
            return *ptrPtr;
        }

        bool IsAssociatedToMapPoint(const Id<MapPoint>& mapPointId) const
        {
            return m_associations.HasMapPoint(mapPointId);
        }

        bool IsAssociatedAtKeypointIndex(KeypointDescriptorIndex index) const
        {
            return m_associations.HasKeypoint(index);
        }

        const cv::KeyPoint& GetKeyPoint(KeypointDescriptorIndex index) const
        {
            return m_image->GetKeyPoint(index);
        }

        void GetAssociatedKeypointDescriptorIndices(std::vector<KeypointDescriptorIndex>& indices) const
        {
            return m_associations.GetAssociatedKeypointDescriptorIndices(indices);
        }

        /// returns the number of common map points between this keyframe and the other
        size_t GetSharedMapPointCount(const Keyframe* pOther) const
        {
            return m_associations.GetSharedMapPointCount(pOther->m_associations);
        }

        void GetMapPoints(std::vector<const MapPoint*>& points) const
        {
            m_associations.GetMapPoints(points);
        }

        void GetMapPoints(temp::vector<const MapPoint*>& points) const
        {
            m_associations.GetMapPoints(points);
        }

        size_t GetMapPointsCount() const
        {
            return m_associations.GetMapPointCount();
        }

        void GetAssociations(std::vector<MapPointAssociations<const MapPoint*>::Association>& associations) const
        {
            return m_associations.GetAssociations(associations);
        }

        void IterateAssociations(const std::function<void(const MapPoint*, KeypointDescriptorIndex)>& iterator) const
        {
            m_associations.IterateAssociations(iterator);
        }

        std::vector<bool> CreateUnassociatedMask() const
        {
            return m_associations.CreateUnassociatedMask();
        }

        size_t GetUnassociatedCount() const
        {
            return m_associations.GetUnassociatedCount();
        }

        bool IsImmortal() const
        {
            return m_immortal;
        }

        void SetImmortal(bool immortal)
        {
            m_immortal = immortal;
        }

        bool IsFixed() const
        {
            return m_fixed;
        }

        void SetFixed(bool fixed)
        {
            m_fixed = fixed;
        }

        void AddDistanceTether(const Id<Keyframe>& originId, float distance, float weight)
        {
            m_tethers.emplace_back(originId, distance, weight);
        }

        void RemoveTethersToKeyframe(const Id<Keyframe>& originId)
        {
            auto newEnd = std::remove_if(m_tethers.begin(), m_tethers.end(), [&originId](const Tether<Keyframe>& tether) { return tether.OriginId() == originId; });
            m_tethers.erase(newEnd, m_tethers.end());
        }

        const std::vector<Tether<Keyframe>>& GetTethers() const
        {
            return m_tethers;
        }

        // image
        const std::shared_ptr<const AnalyzedImage>& GetAnalyzedImage() const
        {
            return m_image;
        }

        template<typename T>
        MapPointAssociations<T> ConvertAssociations() const
        {
            return m_associations.Convert<T>();
        }

    private:
        Id<Keyframe> m_id{};
        std::shared_ptr<const AnalyzedImage> m_image{};                  // holds the image, camera settings, all keypoints and descriptors
        Pose m_pose{};                                                   // calculated camera pose for this frame
        Intrinsics m_undistortedIntrinsics{};                            // focal length and principal point for this keyframe.
        MapPointAssociations<const MapPoint*> m_associations;            // holds the associated and unassociated descriptors
        bool m_immortal{};                                               // used to indicate whether or not this keyframe can be deleted
        bool m_fixed{};                                                  // used to indicate that this Pose is not to be moved by BundleAdjustment
        std::vector<Tether<Keyframe>> m_tethers{};                       // holds the KeyframesTethers, with which we tether the keyframes

        /*
            These two methods are private and should only be called by the
            maps Associate and RemoveAssociation functions.
            The KeypointDescriptorIndex is the index into the AnalyzedImages
            ORBDescriptors and KeyPoints found in this keyframe.
        */
        void AddMapPointAssociation(MapPoint* mappoint, KeypointDescriptorIndex index)
        {
            m_associations.AddAssociation(mappoint, index);
        }

        KeypointDescriptorIndex RemoveMapPointAssociation(MapPoint* mappoint)
        {
            return m_associations.RemoveAssociation(mappoint);
        }

        friend void MapAssociate(Keyframe* keyframe, MapPoint* mappoint, KeypointDescriptorIndex index);
        friend KeypointDescriptorIndex MapRemoveAssociation(Keyframe* keyframe, MapPoint* mappoint);
        friend void MapRemoveAllAssociations(Map* map, MapPoint* mapPoint, std::set<const Keyframe*>& modifiedKeyframes);
    };
}



namespace std
{
    template<>
    struct hash<mage::Id<mage::Keyframe>> : private hash<mage::IdT>
    {
        size_t operator()(const mage::Id<mage::Keyframe>& id) const
        {
            return hash<mage::IdT>::operator()(id.val);
        }
    };
}
