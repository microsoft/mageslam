// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <tuple>

#include "arcana/utils/algorithm.h"

namespace mira
{
    /*
        Helper class for describing a member pointer.
    */
    template<typename ObjT, typename T>
    struct member_pointer
    {
        using pointer_t = T ObjT::*;
        using member_t = T;
        using object_t = ObjT;

        constexpr member_pointer(pointer_t ptr)
            : m_ptr{ ptr }
        {}

        constexpr pointer_t ptr() const
        {
            return m_ptr;
        }

    private:
        const pointer_t m_ptr;
    };

    /*
        Helper function for creating a member object from a pointer to member variable.
    */
    template<typename ObjT, typename T>
    constexpr member_pointer<ObjT, T> make_member(T ObjT::*ptr)
    {
        return { ptr };
    }
}