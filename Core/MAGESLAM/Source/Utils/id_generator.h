// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <atomic>
#include <iostream>

namespace mage
{
    template<typename IdT, typename ScopeT>
    struct id_generator;

    /*
    Opaque id structure that is immutable, type safe
    and scoped to the type of generator (Id<int, ScopeT1> is incompatible with Id<int, ScopeT2>).
    */
    template<typename IdT, typename ScopeT>
    struct id
    {
        using storage_t = IdT;

        id()
            : val{ -1 }
        {}

        /*
        enforce that we can still assign and create a copy
        of an id object.
        */
        id(const id& other) = default;
        id& operator =(const id& other) = default;

        bool IsValid() const { return val != -1; }

        friend std::istream& operator >> (std::istream& stream, id& id)
        {
            return stream >> id.val;
        }

        friend std::ostream& operator << (std::ostream& stream, const id& id)
        {
            return stream << id.val;
        }

    private:

        friend struct id_generator<storage_t, ScopeT>;

        friend class CovisibilityGraph; //allows covisibility graph to wrap/unwrap Id<Keyframes> for use as eigen matrix rows/cols
        friend class SpanningTree;      //allows spanning tree to wrap/unwrap Id<Keyframes> for use as eigen matrix rows/cols

        friend struct std::hash<id>;

        friend bool operator <(const id& left, const id& right)
        {
            return left.val < right.val;
        }

        friend bool operator ==(const id& left, const id& right)
        {
            return left.val == right.val;
        }

        friend bool operator !=(const id& left, const id& right)
        {
            return !(left.val == right.val);
        }

        id(const storage_t& v)
            : val{ v }
        {}

        storage_t val;
    };

    template<typename ArchiveT, typename IdT, typename ScopeT>
    void introspect_object(mira::introspector<ArchiveT>& archive, const id<IdT, ScopeT>& id)
    {
        archive(reinterpret_cast<const IdT&>(id));
    }

    template<typename IdT, typename ScopeT>
    struct id_generator
    {
        id<IdT, ScopeT> generate()
        {
            return{ s_seed.fetch_add(1) };
        }

        static void reset()
        {
            s_seed = 0;
        }

    private:
        static std::atomic<typename id<IdT, ScopeT>::storage_t> s_seed;
    };
}
