// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/utils/serialization/base_stream.h"
#include "arcana/utils/serialization/serializable.h"
#include "arcana/analysis/determinator.h"
#include "Proxies/Proxy.h"
#include "arcana/analysis/binary_iterator.h"
#include "Mapping/MapPointKeyframeAssociations.h"
#include "BundleAdjustment/BundleAdjust.h"
#include "Proxies/KeyframeFields.h"

#include "Tracking/PoseEstimator.h"
#include "Utils/cv.h"

#include <type_traits>
#include <memory>
#include <gsl/gsl>

namespace mage
{
    class MapPoint;
    class Keyframe;

    template<typename IterT>
    inline void binary_iterate(const Id<MapPoint>& id, mira::binary_iterator<IterT>& iterator)
    {
        iterator.iterate(reinterpret_cast<const IdT&>(id));
    }

    template<typename IterT>
    inline void binary_iterate(const Id<Keyframe>& id, mira::binary_iterator<IterT>& iterator)
    {
        iterator.iterate(reinterpret_cast<const IdT&>(id));
    }

    /*
    Serialize only the proxy properties that support serialization
    */
    template<typename IterT, typename T, typename ...Props>
    inline void binary_iterate(const Proxy<T, Props...>& proxy, mira::binary_iterator<IterT>& iterator)
    {
        iterator.iterate(static_cast<const proxy::Id<T>&>(proxy));

        int unused[] = {
            0, // in case it's just a Proxy<MapPoint>, we can't declare an empty c-style array
            (iterator.iterate(static_cast<const Props&>(proxy)) , 0)...
        };
        (void)unused;
    };

    template<typename IterT>
    inline void binary_iterate(const KeyframeReprojection& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(reinterpret_cast<const IdT&>(data.KeyframeId));
        itr.iterate(data.Pose);
        for (auto& point : data.Points)
        {
            itr.iterate(point.As<Proxy<MapPoint, proxy::Position, proxy::UpdateStatistics, proxy::ViewingData>>());
            itr.iterate(*point.GetRepresentativeDescriptor());
        }
    }

    template<typename IterT>
    inline void binary_iterate(const MapPointAssociation& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(&data, sizeof(MapPointAssociation));
    }

    template<typename IterT>
    inline void binary_iterate(const HistoricalFrame& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.UpdatedPose);
        itr.iterate(data.Keyframe);
    }


    template<typename IterT>
    inline void binary_iterate(const AdjustableData& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.Keyframes);
        itr.iterate(data.ExternallyTetheredKeyframes);
        itr.iterate(data.MapPointAssociations);
        itr.iterate(data.MapPoints);
    }

    template<typename IterT>
    inline void binary_iterate(const typename MapPointAssociations<MapPointTrackingProxy>::Association& assoc, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate_all(assoc.MapPoint, assoc.Index);
    }

    template<typename IterT>
    inline void binary_iterate(const MapPointKeyframeAssociations& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.MapPoint);
        itr.iterate(data.Keyframes);
    }

    template<typename IterT>
    inline void binary_iterate(const KeyframeAssociation& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate_all(data.KeyframeId, data.KeypointDescriptorIndex);
    }

    template<typename IterT>
    inline void binary_iterate(const FrameId& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.CorrelationId);
        itr.iterate(data.Camera);
    }

    template<typename IterT>
    inline void binary_iterate(const AnalyzedImage& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.GetFrameId());
        itr.iterate(data.GetTimeStamp().time_since_epoch());
        itr.iterate(data.GetImageData());
        itr.iterate(data.GetDistortedCalibration());
        itr.iterate(data.GetUndistortedCalibration());
        itr.iterate(data.GetKeyPoints());
    }

    template<typename IterT, typename T>
    inline void binary_iterate(const Tether<T>& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.Type());
        itr.iterate(data.Weight());
        itr.iterate(data.OriginId());

        if (data.Type() == TetherType::DISTANCE)
        {
            itr.iterate(data.Distance());
        }

        if(data.Type() == TetherType::SIX_DOF || data.Type() == TetherType::EXTRINSIC)
        {
            itr.iterate(data.Position());
        }

        if (data.Type() == TetherType::THREE_DOF || data.Type() == TetherType::SIX_DOF || data.Type() == TetherType::EXTRINSIC)
        {
            itr.iterate(data.Rotation());
        }
    }

    namespace proxy
    {
        // no-ops that we don't check for determinism
        inline void binary_iterate(const Image&, mira::binary_iterator<mira::determinator::iterator_callback>&) {}
        inline void binary_iterate(const Associations<MapPointTrackingProxy>&, mira::binary_iterator<mira::determinator::iterator_callback>&) {}

        inline void binary_iterate(const UnAssociatedMask& data, mira::binary_iterator<mira::determinator::iterator_callback>& itr)
        {
            itr.iterate(data.GetUnassociatedKeypointCount());
            itr.iterate(data.GetUnassociatedKeypointMask());
        }

        template<typename IterT>
        inline void binary_iterate(const PoseConstraints& data, mira::binary_iterator<IterT>& itr)
        {
            itr.iterate(data.IsImmortal());
            itr.iterate(data.IsFixed());
            itr.iterate(data.GetTethers());
        }

        template<typename IterT>
        inline void binary_iterate(const Descriptor& data, mira::binary_iterator<IterT>& itr)
        {
            // proxy::Descriptor is just a pointer to the descriptor
            itr.iterate(data.GetRepresentativeDescriptor()->Data(), mage::ORBDescriptor::DESCRIPTOR_SIZE_BYTES);
        }

        template<typename IterT>
        inline void binary_iterate(const Associations<mage::MapPointTrackingProxy>& data, mira::binary_iterator<IterT>& itr)
        {
            // not the real size as we don't account for the bimap overhead
            std::vector<mage::MapPointAssociations<mage::MapPointTrackingProxy>::Association> assocs;
            data.GetAssociations(assocs);
            itr.iterate(assocs);
        }
    }
}

namespace cv
{
    template<typename IterT>
    inline void binary_iterate(const cv::KeyPoint& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(&data, sizeof(data));
    }

    template<typename IterT>
    inline void binary_iterate(const cv::Vec3f& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(&data, sizeof(data));
    }
}


namespace Eigen
{
    template<typename IterT, typename T>
    inline void binary_iterate(const Quaternion<T>& data, mira::binary_iterator<IterT>& itr)
    {
        itr.iterate(data.coeffs().data(), data.coeffs().size()*sizeof(T));
    }
}
