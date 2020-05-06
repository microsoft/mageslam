// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Utils/cv.h"

#include <arcana/analysis/introspector.h>

namespace mage
{
    enum class TetherType
    {
        DISTANCE,
        THREE_DOF,
        SIX_DOF,
        EXTRINSIC,
    };

    /// This class is intended to represent a spatial relationship between two entities, such as a known
    /// and unvarying distance or rotation between two keyframes.  The Tether stores the identifier of
    /// the second entity in the relationship (the first entity, implicitly, is the owner of the Tether)
    /// as well as such spatial information as is required to fully describe the tethering relationship.
    template <typename T>
    class Tether
    {
    public:
        Tether(const Id<T>& originId, const cv::Vec3f& pos, const Quaternion& rot, float weight, TetherType type)
            : m_originId{ originId }, m_position { pos }, m_rotation{ rot }, m_weight{ weight }, m_type{ type }
        {}

        Tether(const Id<T>& originId, float distance, float weight)
            : Tether<T>(originId, { distance, 0.f, 0.f }, {}, weight, TetherType::DISTANCE)
        {}

        Tether(const Id<T>& originId, const Quaternion& rotation, float weight)
            : Tether<T>(originId, {}, rotation, weight, TetherType::THREE_DOF)
        {}

        const Id<T>& OriginId() const
        {
            return m_originId;
        }

        float Distance() const
        {
            assert(m_type == TetherType::DISTANCE);
            return m_position[0];
        }

        const cv::Vec3f& Position() const
        {
            assert(m_type == TetherType::SIX_DOF || m_type == TetherType::EXTRINSIC);
            return m_position;
        }

        const Quaternion& Rotation() const
        {
            assert(m_type == TetherType::THREE_DOF || m_type == TetherType::SIX_DOF || m_type == TetherType::EXTRINSIC);
            return m_rotation;
        }

        float Weight() const
        {
            return m_weight;
        }

        TetherType Type() const
        {
            return m_type;
        }

        const cv::Matx44f TransformMatrix() const
        {
            auto rotation = ToMat(m_rotation);
            return {
                rotation(0, 0),     rotation(0, 1),     rotation(0, 2),     m_position(0),
                rotation(1, 0),     rotation(1, 1),     rotation(1, 2),     m_position(1),
                rotation(2, 0),     rotation(2, 1),     rotation(2, 2),     m_position(2),
                0.f,            0.f,            0.f,         1.f,
            };
        }

    private:
        Id<T> m_originId{};
        cv::Vec3f m_position{};
        Quaternion m_rotation{};
        float m_weight{};
        TetherType m_type{};
    };

    template<typename ArchiveT, typename T>
    void introspect_object(mira::introspector<ArchiveT>& intro, const Tether<T>& tether)
    {
        intro(
            cereal::make_nvp("OriginId", tether.OriginId()),
            cereal::make_nvp("Type", tether.Type())
        );
    }
}
