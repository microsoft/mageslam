// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Types.h"

#include "Image\AnalyzedImage.h"

#include <functional>

namespace mage
{
    class MapPoint;

    /*
        Class responsible for dealing with MapPoints/ORBDescriptors/Keypoints associations.
    */
    template<typename MapPointT>
    class MapPointAssociations
    {
        struct Impl;
    public:
        struct Association
        {
            MapPointT MapPoint;

            /*
                the index of the descriptor and keypoint
                associated to this MapPoint
            */
            KeypointDescriptorIndex Index;

            Association(const MapPointT& mp, KeypointDescriptorIndex idx)
                : MapPoint(mp), Index(idx)
            {}
        };

        template<typename ArchiveT>
        friend void introspect_object(mira::introspector<ArchiveT>& intro, Association assoc)
        {
            intro(
                cereal::make_nvp("MapPoint", assoc.MapPoint.GetId()),
                cereal::make_nvp("Index", assoc.Index)
            );
        }

        MapPointAssociations(const std::shared_ptr<const AnalyzedImage>& image, const std::vector<Association>& associations);
        MapPointAssociations(const std::shared_ptr<const AnalyzedImage>& image);
        ~MapPointAssociations();

        /*
            Only implement the move operators, in order to avoid having people
            copy this object. The Clone method should be used if a copy is needed.
        */
        MapPointAssociations& operator=(MapPointAssociations&&) noexcept;
        MapPointAssociations(MapPointAssociations&&) noexcept;

        MapPointAssociations& operator=(const MapPointAssociations&);
        MapPointAssociations(const MapPointAssociations&);

        size_t GetMapPointCount() const;
        
        void GetMapPoints(std::vector<MapPointT>& points) const;
        void GetMapPoints(temp::vector<MapPointT>& points) const;
        void GetMapPoints(loop::vector<MapPointT>& points) const;

        bool HasMapPoint(const Id<MapPoint>& id) const;
        bool HasKeypoint(const KeypointDescriptorIndex idx) const;

        void GetAssociatedKeypointDescriptorIndices(std::vector<KeypointDescriptorIndex>& indices) const;
        void GetAssociations(std::vector<Association>& associations) const;
        void IterateAssociations(const std::function<void(const MapPointT&, KeypointDescriptorIndex)>& iterator) const;
        void IterateAssociations(const std::function<void(MapPointT&, KeypointDescriptorIndex)>& iterator);

        // descriptors
        const ORBDescriptor& GetAssociatedDescriptor(const MapPointT& mp) const;

        // keypoints
        const cv::KeyPoint& GetAssociatedKeyPoint(const MapPointT& mp) const;

        const MapPointT* GetAssociatedMapPoint(KeypointDescriptorIndex index) const;

        // general
        KeypointDescriptorIndex GetAssociatedIndex(const MapPointT& mapPoint) const;
        KeypointDescriptorIndex GetAssociatedIndex(const Id<MapPoint>& mapPoint) const;

        /*
        returns a mask vector where indexes set to true
        are unassociated.
        */
        std::vector<bool> CreateUnassociatedMask() const;
        temp::vector<bool> CreateUnassociatedMask(thread_memory& memory) const;

        /*
        returns a mask vector where indexes set to true
        are associated.
        */
        std::vector<bool> CreateAssociatedMask() const;

        /*
            returns the number of descriptors that haven't been associated yet.
        */
        size_t GetUnassociatedCount() const;
        
        /*
        returns the number of descriptors that have been associated
        */
        size_t GetAssociatedCount() const;

        // add remove functions
        void AddAssociation(const MapPointT& mapPoint, KeypointDescriptorIndex association);
        KeypointDescriptorIndex RemoveAssociation(const MapPointT& mapPoint);
        KeypointDescriptorIndex RemoveAssociation(const Id<MapPoint>& mapPoint);
        bool TryRemoveAssociation(const Id<MapPoint>& id);
        void ClearAssociations();

        size_t GetSharedMapPointCount(const MapPointAssociations& other) const;

        template<typename OtherMapPointT>
        MapPointAssociations<OtherMapPointT> Convert() const;
    private:
        std::unique_ptr<Impl> m_impl;

        MapPointAssociations(std::unique_ptr<Impl>&& impl);

        template<typename OtherT>
        friend class MapPointAssociations;
    };

    class MapPoint;

    // declare the template instantiations that this class supports
    extern template MapPointAssociations<MapPoint const*>;
}
