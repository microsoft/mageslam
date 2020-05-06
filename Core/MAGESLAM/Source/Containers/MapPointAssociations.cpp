// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "MapPointAssociations.h"
#include "Proxies\MapPointProxy.h"

#include "Map\MapPoint.h"
#include "arcana\utils\algorithm.h"

#include <boost\bimap.hpp>
#include <boost\bimap\vector_of.hpp>

#include <memory>
#include <vector>

using namespace std;

namespace mage
{
    template<typename MapPointT>
    struct MapPointAssociations<MapPointT>::Impl
    {
        using BiMap = boost::bimap<
            boost::bimaps::set_of<boost::bimaps::tagged<Id<MapPoint>, Id<MapPoint>>>,
            boost::bimaps::set_of<boost::bimaps::tagged<KeypointDescriptorIndex, KeypointDescriptorIndex>>,
            boost::bimaps::with_info<boost::bimaps::tagged<MapPointT, MapPointT>>
        >;

        shared_ptr<const AnalyzedImage> m_image;
        BiMap m_associations;

        Impl(const shared_ptr<const AnalyzedImage>& image)
            : m_image{ image }
        {}

        Impl(shared_ptr<const AnalyzedImage>& image, const BiMap& map)
            : m_image{ image }, m_associations{map}
        {}
    };

    template<typename MapPointT>
    MapPointAssociations<MapPointT>::MapPointAssociations(
        const shared_ptr<const AnalyzedImage>& image,
        const std::vector<Association>& associations)
        :   m_impl{ make_unique<Impl>(image)}
    {
        for (const auto& assoc : associations)
        {
            m_impl->m_associations.insert(Impl::BiMap::value_type{ fetch_id(assoc.MapPoint), assoc.Index, assoc.MapPoint });
        }
    }

    template<typename MapPointT>
    MapPointAssociations<MapPointT>::MapPointAssociations(const std::shared_ptr<const AnalyzedImage>& image)
        : m_impl{ make_unique<Impl>(image) }
    {}

    template<typename MapPointT>
    MapPointAssociations<MapPointT>::MapPointAssociations(std::unique_ptr<Impl>&& impl)
        : m_impl{move(impl)}
    {}

    template<typename MapPointT>
    MapPointAssociations<MapPointT>& MapPointAssociations<MapPointT>::operator=(MapPointAssociations<MapPointT>&& other) noexcept
    {
        m_impl = move(other.m_impl);

        return *this;
    }

    template<typename MapPointT>
    MapPointAssociations<MapPointT>::MapPointAssociations(MapPointAssociations<MapPointT>&& other) noexcept
        : m_impl{move(other.m_impl)}
    {}

    template<typename MapPointT>
    MapPointAssociations<MapPointT>& MapPointAssociations<MapPointT>::operator=(const MapPointAssociations& other)
    {
        m_impl = make_unique<Impl>(*other.m_impl);
        return *this;
    }

    template<typename MapPointT>
    MapPointAssociations<MapPointT>::MapPointAssociations(const MapPointAssociations& other)
        : m_impl{ make_unique<Impl>(*other.m_impl) }
    {}

    template<typename MapPointT>
    MapPointAssociations<MapPointT>::~MapPointAssociations()
    {}

    template<typename MapPointT>
    size_t MapPointAssociations<MapPointT>::GetMapPointCount() const
    {
        return m_impl->m_associations.size();
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::GetMapPoints(std::vector<MapPointT>& points) const
    {
        points.reserve(m_impl->m_associations.size());
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            points.emplace_back(assoc.get<MapPointT>());
        }
    }

    template<typename MapPointT>
    bool MapPointAssociations<MapPointT>::HasMapPoint(const Id<MapPoint>& id) const
    {
        auto container = m_impl->m_associations.by<Id<MapPoint>>();
        return container.find(id) != container.end();
    }

    template<typename MapPointT>
    bool MapPointAssociations<MapPointT>::HasKeypoint(const KeypointDescriptorIndex idx) const
    {
        auto container = m_impl->m_associations.by<KeypointDescriptorIndex>();
        return container.find(idx) != container.end();
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::GetMapPoints(temp::vector<MapPointT>& points) const
    {
        points.reserve(m_impl->m_associations.size());
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            points.emplace_back(assoc.get<MapPointT>());
        }
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::GetMapPoints(loop::vector<MapPointT>& points) const
    {
        points.reserve(m_impl->m_associations.size());
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            points.emplace_back(assoc.get<MapPointT>());
        }
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::GetAssociatedKeypointDescriptorIndices(std::vector<KeypointDescriptorIndex>& indices) const
    {
        indices.reserve(m_impl->m_associations.size());
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            indices.emplace_back(assoc.get<KeypointDescriptorIndex>());
        }
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::GetAssociations(std::vector<Association>& associations) const
    {
        associations.reserve(m_impl->m_associations.size());
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            associations.emplace_back(assoc.get<MapPointT>(), assoc.get<KeypointDescriptorIndex>());
        }
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::IterateAssociations(const std::function<void(const MapPointT&, KeypointDescriptorIndex)>& iterator) const
    {
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            iterator(assoc.get<MapPointT>(), assoc.get<KeypointDescriptorIndex>());
        }
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::IterateAssociations(const std::function<void(MapPointT&, KeypointDescriptorIndex)>& iterator)
    {
        for (auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            iterator(assoc.get<MapPointT>(), assoc.get<KeypointDescriptorIndex>());
        }
    }

    namespace
    {
        inline bool equals(MapPoint const* left, MapPoint const* right)
        {
            return left->GetId() == right->GetId();
        }

        inline bool equals(const proxy::Id<MapPoint>& left, const proxy::Id<MapPoint>& right)
        {
            return left.GetId() == right.GetId();
        }

        inline Id<MapPoint> fetch_id(MapPoint const* mp)
        {
            return mp->GetId();
        }

        inline Id<MapPoint> fetch_id(const proxy::Id<MapPoint>& mp)
        {
            return mp.GetId();
        }
    }

    template<typename MapPointT>
    const ORBDescriptor& MapPointAssociations<MapPointT>::GetAssociatedDescriptor(const MapPointT& mapPoint) const
    {
        auto idx = m_impl->m_associations.by<Id<MapPoint>>().find(fetch_id(mapPoint))->get<KeypointDescriptorIndex>();
        return m_impl->m_image->GetDescriptor(idx);
    }
    
    template<typename MapPointT>
    const cv::KeyPoint& MapPointAssociations<MapPointT>::GetAssociatedKeyPoint(const MapPointT& mapPoint) const
    {
        auto itr = m_impl->m_associations.by<Id<MapPoint>>().find(fetch_id(mapPoint));
        auto kpindex = itr->get<KeypointDescriptorIndex>();
        return m_impl->m_image->GetKeyPoint(kpindex);
    }

    template<typename MapPointT>
    KeypointDescriptorIndex MapPointAssociations<MapPointT>::GetAssociatedIndex(const MapPointT& mapPoint) const
    {
        auto itr = m_impl->m_associations.by<Id<MapPoint>>().find(fetch_id(mapPoint));
        return itr->get<KeypointDescriptorIndex>();
    }

    template<typename MapPointT>
    KeypointDescriptorIndex MapPointAssociations<MapPointT>::GetAssociatedIndex(const Id<MapPoint>& id) const
    {
        auto itr = m_impl->m_associations.by<Id<MapPoint>>().find(id);
        return itr->get<KeypointDescriptorIndex>();
    }

    template<typename MapPointT>
    const MapPointT* MapPointAssociations<MapPointT>::GetAssociatedMapPoint(KeypointDescriptorIndex index) const
    {
        auto container = m_impl->m_associations.by<KeypointDescriptorIndex>();
        auto itr = container.find(index);
        if (itr != container.end())
        {
            return &itr->get<MapPointT>();
        }
        return nullptr;
    }

    template<typename MapPointT>
    std::vector<bool> MapPointAssociations<MapPointT>::CreateUnassociatedMask() const
    {
        vector<bool> flags(m_impl->m_image->GetDescriptorsCount(), true);
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            flags[assoc.get<KeypointDescriptorIndex>()] = false;
        }
        return flags;
    }

    template<typename MapPointT>
    temp::vector<bool> MapPointAssociations<MapPointT>::CreateUnassociatedMask(thread_memory& memory) const
    {
        temp::vector<bool> flags = memory.stack_vector<bool>();
        flags.resize(m_impl->m_image->GetDescriptorsCount(), true);
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            flags[assoc.get<KeypointDescriptorIndex>()] = false;
        }
        return flags;
    }

    template<typename MapPointT>
    std::vector<bool> MapPointAssociations<MapPointT>::CreateAssociatedMask() const
    {
        vector<bool> flags(m_impl->m_image->GetDescriptorsCount(), false);
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            flags[assoc.get<KeypointDescriptorIndex>()] = true;
        }
        return flags;
    }

    template<typename MapPointT>
    size_t MapPointAssociations<MapPointT>::GetAssociatedCount() const
    {
        return GetMapPointCount();
    }

    template<typename MapPointT>
    size_t MapPointAssociations<MapPointT>::GetUnassociatedCount() const
    {
        return m_impl->m_image->GetDescriptorsCount() - GetMapPointCount();
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::AddAssociation(const MapPointT& mapPoint, KeypointDescriptorIndex association)
    {
        assert(m_impl->m_associations.by<Id<MapPoint>>().find(fetch_id(mapPoint)) == m_impl->m_associations.by<Id<MapPoint>>().end());
        assert(m_impl->m_associations.by<KeypointDescriptorIndex>().find(association) == m_impl->m_associations.by<KeypointDescriptorIndex>().end());

        m_impl->m_associations.insert(Impl::BiMap::value_type{ fetch_id(mapPoint), association, mapPoint });

        assert(m_impl->m_associations.by<Id<MapPoint>>().find(fetch_id(mapPoint)) != m_impl->m_associations.by<Id<MapPoint>>().end());
        assert(m_impl->m_associations.by<KeypointDescriptorIndex>().find(association) != m_impl->m_associations.by<KeypointDescriptorIndex>().end());
    }

    template<typename MapPointT>
    void MapPointAssociations<MapPointT>::ClearAssociations()
    {
        m_impl->m_associations.clear();
    }

    template<typename MapPointT>
    KeypointDescriptorIndex MapPointAssociations<MapPointT>::RemoveAssociation(const MapPointT& mapPoint)
    {
        auto container = m_impl->m_associations.by<Id<MapPoint>>();
        auto itr = container.find(fetch_id(mapPoint));
        assert(itr != container.end());
        KeypointDescriptorIndex index = itr->get<KeypointDescriptorIndex>();
        container.erase(itr);
        
        return index;
    }

    template<typename MapPointT>
    KeypointDescriptorIndex MapPointAssociations<MapPointT>::RemoveAssociation(const Id<MapPoint>& id)
    {
        auto container = m_impl->m_associations.by<Id<MapPoint>>();
        auto itr = container.find(id);
        assert(itr != container.end());
        KeypointDescriptorIndex index = itr->get<KeypointDescriptorIndex>();
        container.erase(itr);
        
        return index;
    }

    template<typename MapPointT>
    bool MapPointAssociations<MapPointT>::TryRemoveAssociation(const Id<MapPoint>& id)
    {
        auto container = m_impl->m_associations.by<Id<MapPoint>>();
        auto itr = container.find(id);
        if (itr != container.end())
        {
            container.erase(itr);
            return true;
        }
        return false;
    }

    template<typename MapPointT>
    template<typename OtherMapPointT>
    MapPointAssociations<OtherMapPointT> MapPointAssociations<MapPointT>::Convert() const
    {
        MapPointAssociations<OtherMapPointT> other{ m_impl->m_image };
        for (const auto& assoc : m_impl->m_associations.by<Id<MapPoint>>())
        {
            other.AddAssociation(OtherMapPointT{ assoc.get<MapPointT>() }, assoc.get<KeypointDescriptorIndex>());
        }
        return move(other);
    }

    template<typename MapPointT>
    size_t MapPointAssociations<MapPointT>::GetSharedMapPointCount(const MapPointAssociations& other) const
    {
        auto myMapPoints = m_impl->m_associations.by<Id<MapPoint>>();
        auto otherMapPoints = other.m_impl->m_associations.by<Id<MapPoint>>();

        size_t count = count_if(myMapPoints.begin(), myMapPoints.end(),
            [&otherMapPoints](const auto& assoc)
            {
                return otherMapPoints.find(assoc.get<Id<MapPoint>>()) != otherMapPoints.end();
            });

        // TODO PERF:  this code really should be faster than the above, but we've had no luck in making actually be so.
        //size_t count = set_intersection_count(
        //    myMapPoints.begin(), myMapPoints.end(),
        //    otherMapPoints.begin(), otherMapPoints.end());

        return count;
    }

    // instantiate the template definitions that this class supports
    template MapPointAssociations<MapPoint const*>;

    template MapPointAssociations<Proxy<MapPoint>>;
    template MapPointAssociations<MapPointProxy>;
    template MapPointAssociations<MapPointTrackingProxy>;

    template MapPointAssociations<Proxy<MapPoint>> MapPointAssociations<MapPoint const*>::Convert<Proxy<MapPoint>>() const;
    template MapPointAssociations<MapPointProxy> MapPointAssociations<MapPoint const*>::Convert<MapPointProxy>() const;
    template MapPointAssociations<MapPointTrackingProxy> MapPointAssociations<MapPoint const*>::Convert<MapPointTrackingProxy>() const;
    template MapPointAssociations<Proxy<MapPoint>> MapPointAssociations<MapPointTrackingProxy>::Convert<Proxy<MapPoint>>() const;
}
