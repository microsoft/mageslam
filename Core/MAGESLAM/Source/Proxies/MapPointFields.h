// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "GenericFields.h"

#include "Data\Types.h"
#include "Map\MapPoint.h"

#include "arcana\utils\serialization\serializable.h"

namespace mage
{
    namespace proxy
    {
        struct Position : mira::serializable<Position>
        {
            Position(const MapPoint& original)
                : m_prop{original.GetPosition()}
            {}

            Position(const cv::Point3f& value)
                : m_prop{ value }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            Position(StreamT& stream)
            {
                deserialize(stream);
            }

            static constexpr auto members()
            {
                return declare_members(
                    &Position::m_prop
                );
            }

            const cv::Point3f& GetPosition() const
            {
                return m_prop;
            }

            void SetPosition(const cv::Point3f& value)
            {
                m_prop = value;
            }

            void SetPosition(float x, float y, float z)
            {
                m_prop.x = x;
                m_prop.y = y;
                m_prop.z = z;
            }
        private:
            cv::Point3f m_prop;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const Position& proxy)
        {
            intro(
                cereal::make_nvp("Position", gsl::make_span(&proxy.GetPosition().x, 3))
            );
        }

        struct Descriptor
        {
            Descriptor(const MapPoint& original)
                : m_prop{ original.GetRepresentativeDescriptor().AsRef() }
            {}

            Descriptor(const mage::ORBDescriptor::Ref& value)
                : m_prop{ value }
            {}

            const ORBDescriptor::Ref& GetRepresentativeDescriptor() const
            {
                return m_prop;
            }
        private:
            mage::ORBDescriptor::Ref m_prop;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>&, const Descriptor&)
        {}

        struct DescriptorCopy
        {
            DescriptorCopy(const MapPoint& original)
                : DescriptorCopy{ original.GetRepresentativeDescriptor() }
            {}

            DescriptorCopy(const mage::ORBDescriptor& value)
            {
                m_prop.CopyFrom(value);
            }

            DescriptorCopy(const DescriptorCopy& value)
                : DescriptorCopy{value.m_prop}
            {}

            DescriptorCopy& operator =(const DescriptorCopy& other)
            {
                m_prop.CopyFrom(other.m_prop);
                return *this;
            }

            const ORBDescriptor& GetRepresentativeDescriptor() const
            {
                return m_prop;
            }
        private:
            mage::ORBDescriptor m_prop;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>&, const DescriptorCopy&)
        {}

        struct UpdateStatistics : mira::serializable<UpdateStatistics>
        {
            UpdateStatistics(const MapPoint& original)
                : m_refinementCount{ original.GetRefinementCount() }
            {}

            explicit UpdateStatistics(unsigned int refinementCount)
                : m_refinementCount{ refinementCount }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            UpdateStatistics(StreamT& stream)
            {
                deserialize(stream);
            }

            static constexpr auto members()
            {
                return declare_members(
                    &UpdateStatistics::m_refinementCount
                );
            }

            unsigned int GetRefinementCount() const
            {
                return m_refinementCount;
            }

            void IncrementRefinementCount()
            {
                m_refinementCount++;
            }

        private:
            unsigned int m_refinementCount;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const UpdateStatistics& proxy)
        {
            intro(
                cereal::make_nvp("RefinementCount", proxy.GetRefinementCount())
            );
        }

        struct ViewingData : mira::serializable<ViewingData>
        {
            ViewingData(const MapPoint& original)
                :   m_meanViewingDirection{ original.GetMeanViewingDirection() },
                    m_dMin{ original.GetDMin() },
                    m_dMax{ original.GetDMax() }
            {}

            ViewingData(const cv::Vec3f& meanViewingDir, float dmin, float dmax)
                :   m_meanViewingDirection{ meanViewingDir },
                    m_dMin{ dmin },
                    m_dMax{ dmax }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            ViewingData(StreamT& stream)
            {
                deserialize(stream);
            }

            static constexpr auto members()
            {
                return declare_members(
                    &ViewingData::m_meanViewingDirection,
                    &ViewingData::m_dMin,
                    &ViewingData::m_dMax
                );
            }

            const cv::Vec3f& GetMeanViewingDirection() const
            {
                return m_meanViewingDirection;
            }

            float GetDMin() const
            {
                return m_dMin;
            }

            float GetDMax() const
            {
                return m_dMax;
            }
        private:
            cv::Vec3f m_meanViewingDirection;
            float m_dMin;
            float m_dMax;
        };


        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const ViewingData& proxy)
        {
            intro(
                cereal::make_nvp("MeanViewingDirection", gsl::make_span(proxy.GetMeanViewingDirection().val)),
                cereal::make_nvp("DMin", proxy.GetDMin()),
                cereal::make_nvp("DMax", proxy.GetDMax())
            );
        }

    }
}
