// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "base_stream.h"
#include "member_serializer.h"

namespace mira
{
    /*
        Empty helper base class to serialize and deserialize objects to a stream.

        T needs to satisfy this contract:
            - a members() function that returns a tuple of pointer to members
            that should be serialized/deserialized.

                ex:
                    static constexpr auto members()
                    {
                        return declare_members(
                            &foo::member1,
                            &foo::member2,
                        );
                    }

            - a constructor accepting a templated StreamT& parameter that
            only calls the serializable::deserialize function (for RAII initialization)

                ex:
                    template<typename StreamT, typename = is_stream_t<StreamT>>
                    explicit foo(StreamT& stream)
                    {
                        deserialize(stream);
                    }
    */
    template<typename T>
    class serializable
    {
    public:
        template<typename StreamT>
        void serialize(StreamT& stream) const
        {
            member_serializer<T>::write_to(stream, *static_cast<const T*>(this));
        }

    protected:
        /*
            Helper function that needs to be called from the constructor
            accepting a StreamT& parameter
        */
        template<typename StreamT>
        void deserialize(StreamT& stream)
        {
            member_serializer<T>::read_from(stream, *static_cast<T*>(this));
        }

        /*
            Helper function for declaring T's members in the members() function
        */
        template<typename... MemberPointers>
        static constexpr auto declare_members(MemberPointers... members)
        {
            return std::make_tuple(make_member(members)...);
        }
    };

    template<typename StreamT>
    using is_stream_t = std::enable_if_t<std::is_base_of<base_stream, StreamT>::value>;
}