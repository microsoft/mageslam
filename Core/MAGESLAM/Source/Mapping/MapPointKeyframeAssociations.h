// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Types.h"

#include "Containers\MapPointAssociations.h"
#include "Proxies\MapPointProxy.h"
#include "Data\Pose.h"

#include <arcana/analysis/introspector.h>

#include <vector>

namespace mage
{
    class Keyframe;

    /*
        Represents a keyframe that is associated with
        a MapPoint.
    */
    struct KeyframeAssociation
    {
        Id<Keyframe> KeyframeId;
        KeypointDescriptorIndex KeypointDescriptorIndex;
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const KeyframeAssociation& object)
    {
        intro(
            cereal::make_nvp("KeyframeId", object.KeyframeId),
            cereal::make_nvp("KeypointDescriptorIndex", object.KeypointDescriptorIndex)
        );
    }

    /*
        Represents all the keyframes associated
        with a MapPoint.
    */
    struct MapPointKeyframeAssociations
    {
        MapPointProxy MapPoint;
        std::vector<KeyframeAssociation> Keyframes;
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const MapPointKeyframeAssociations& object)
    {
        intro(
            cereal::make_nvp("MapPoint", object.MapPoint.GetId()),
            cereal::make_nvp("Keyframes", object.Keyframes)
        );
    }

    /*
        Represents all the map points associated
        with a keyframe.
    */
    struct KeyframeReprojection
    {
        Id<Keyframe> KeyframeId;
        Pose Pose;
        std::vector<MapPointProxy> Points;
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const KeyframeReprojection& object)
    {
        intro(
            cereal::make_nvp("KeyframeId", object.KeyframeId),
            cereal::make_nvp("Pose", object.Pose),
            cereal::make_nvp("Points", object.Points)
        );
    }

    /*
        Represents a map point that is associated to a
        keyframe.
    */
    struct MapPointAssociation
    {
        cv::Point2f Projection;
        Id<MapPoint> MapPointId;
        Id<Keyframe> KeyframeId;

        MapPointAssociation(const cv::Point2f& projection, const Id<MapPoint>& mapPointId, const Id<Keyframe>& keyframeId)
            : Projection { projection },
            MapPointId { mapPointId },
            KeyframeId { keyframeId }
        {
        }
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const MapPointAssociation& data)
    {
        intro(
            cereal::make_nvp("Projection", gsl::make_span(&data.Projection.x, 2)),
            cereal::make_nvp("MapPointId", data.MapPointId),
            cereal::make_nvp("KeyframeId", data.KeyframeId)
        );
    }
}
