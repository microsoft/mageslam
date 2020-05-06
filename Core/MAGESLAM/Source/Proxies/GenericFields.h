// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <arcana/utils/serialization/serializable.h>
#include <arcana/analysis/introspector.h>

#include "Data/Types.h"

namespace mage
{
    namespace proxy
    {
        template<typename T>
        struct Id : mira::serializable<Id<T>>
        {
            Id(const T& original)
                : m_id{ original.GetId() }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            Id(StreamT& stream)
            {
                deserialize(stream);
            }

            static constexpr auto members()
            {
                return declare_members(
                    &Id::m_id
                );
            }

            const mage::Id<T>& GetId() const
            {
                return m_id;
            }

        protected:
            Id(const mage::Id<T>& id)
                : m_id{ id }
            {}

        private:
            mage::Id<T> m_id;
        };

        template<typename ArchiveT, typename T>
        void introspect_object(mira::introspector<ArchiveT>& archive, const Id<T>& id)
        {
            archive(
                cereal::make_nvp("Id", id.GetId())
            );
        }

        template<typename T>
        inline bool operator <(const Id<T>& left, const Id<T>& right)
        {
            return left.GetId() < right.GetId();
        }

        template<typename T>
        inline bool operator ==(const Id<T>& left, const Id<T>& right)
        {
            return left.GetId() == right.GetId();
        }

        template<typename T>
        inline bool operator !=(const Id<T>& left, const Id<T>& right)
        {
            return !(left.GetId() == right.GetId());
        }
    }
}
