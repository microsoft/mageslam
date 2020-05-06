// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "custom_serialization.h"
#include "member.h"

#include <tuple>

namespace mira
{
    template<typename>
    class serializable;

    /*
        Serializes and Deserializes an objects member variables.

        ObjT needs to satisfy this contract:
            - a members() function that returns a tuple of pointer to members
            that should be serialized/deserialized.

        StreamT needs to satisfy this contract:
            - a write function that takes a parameter by const&
            and writes it out to the stream
            - a read function that takes a parameter by &
            and populates it from its stream
    */
    template<typename ObjT>
    struct member_serializer
    {
        using object_t = ObjT;
        using member_tuple_t = decltype(object_t::members());
        using indexing = build_indices<std::tuple_size<member_tuple_t>::value>;

        /*
            reads all the member variables from a stream and assigns them
            to obj.
        */
        template<typename StreamT>
        static void read_from(StreamT& stream, object_t& obj)
        {
            read_from(stream, obj, indexing{});
        }

        /*
            writes out all the member variables of obj to a stream.
        */
        template<typename StreamT>
        static void write_to(StreamT& stream, const object_t& obj)
        {
            write_to(stream, obj, indexing{});
        }

    private:
        /*
            Functions that apply the expansion to serialize every
            member of an object
        */
        template<typename StreamT, size_t... Index>
        static void read_from(StreamT& stream, object_t& obj, indices<Index...>)
        {
            constexpr auto members = object_t::members();
            int unused[] = { (read_from(stream, obj, std::get<Index>(members)), 0)... };
            (void)unused;
        }

        template<typename StreamT, size_t... Index>
        static void write_to(StreamT& stream, const object_t& obj, indices<Index...>)
        {
            constexpr auto members = object_t::members();
            int unused[] = { (write_to(stream, obj, std::get<Index>(members)), 0)... };
            (void)unused;
        }

        /*
            Helper functions to avoid putting more complexity in the expansion code above.
        */

        template<typename MemberT>
        using is_serializable_t =
            std::enable_if_t<std::is_base_of<serializable<std::remove_cv_t<MemberT>>, MemberT>::value>;

        template<typename MemberT>
        using not_serializable_t =
            std::enable_if_t<!std::is_base_of<serializable<std::remove_cv_t<MemberT>>, MemberT>::value>;

        template<typename StreamT, typename MemberT>
        static not_serializable_t<MemberT> read_from(StreamT& stream,
                                                     object_t& obj,
                                                     const member_pointer<object_t, MemberT>& member)
        {
            using member_t = std::remove_cv_t<typename member_pointer<object_t, MemberT>::member_t>;
            custom_serialization<member_t>::read(stream, const_cast<member_t&>(obj.*(member.ptr())));
        }

        template<typename StreamT, typename MemberT>
        static not_serializable_t<MemberT> write_to(StreamT& stream,
                                                    const object_t& obj,
                                                    const member_pointer<object_t, MemberT>& member)
        {
            using member_t = std::remove_cv_t<typename member_pointer<object_t, MemberT>::member_t>;
            custom_serialization<member_t>::write(stream, const_cast<member_t&>(obj.*(member.ptr())));
        }

        template<typename StreamT, typename MemberT>
        static is_serializable_t<MemberT> read_from(StreamT& stream,
                                                    object_t& obj,
                                                    const member_pointer<object_t, MemberT>& member)
        {
            member_serializer<std::remove_cv_t<MemberT>>::read_from(
                stream,
                const_cast<std::remove_cv_t<typename member_pointer<object_t, MemberT>::member_t>&>(obj.*
                                                                                                    (member.ptr())));
        }

        template<typename StreamT, typename MemberT>
        static is_serializable_t<MemberT> write_to(StreamT& stream,
                                                   const object_t& obj,
                                                   const member_pointer<object_t, MemberT>& member)
        {
            member_serializer<std::remove_cv_t<MemberT>>::write_to(stream, obj.*(member.ptr()));
        }
    };
}
