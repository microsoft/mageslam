// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Memory/allocators.h"

#include "Utils/buffer.h"
#include "arcana/containers/sorted_vector.h"
#include "arcana/containers/unique_vector.h"

#include <vector>
#include <set>
#include <unordered_set>
#include <unordered_map>

namespace mage
{
    namespace temp
    {
        template<typename T>
        using vector = std::vector<T, stack_allocator<T>>;

        template<typename KeyT, typename CompareT = std::less<KeyT>>
        using set = std::set<KeyT, CompareT, stack_allocator<KeyT>>;

        template<typename KeyT, typename HashT = std::hash<KeyT>, typename KeyEqT = std::equal_to<KeyT>>
        using unordered_set = std::unordered_set<KeyT, HashT, KeyEqT, stack_allocator<KeyT>>;

        template<typename KeyT, typename ValT, typename HashT = std::hash<KeyT>, typename KeyEqT = std::equal_to<KeyT>>
        using unordered_map = std::unordered_map<KeyT, ValT, HashT, KeyEqT, stack_allocator<std::pair<const KeyT, ValT>>>;

        template<typename T>
        using buffer = mage::buffer<T, stack_allocator<T>>;

        template<typename KeyT, typename CompareT = std::less<KeyT>>
        using sorted_vector = mira::sorted_vector<KeyT, CompareT, stack_allocator<KeyT>>;

        template<typename KeyT, typename CompareT = std::less<KeyT>>
        using unique_vector = mira::unique_vector<KeyT, CompareT, stack_allocator<KeyT>>;
    }

    namespace loop
    {
        template<typename T>
        using vector = std::vector<T, loop_allocator<T>> ;

        template<typename KeyT, typename CompareT = std::less<KeyT>>
        using set = std::set<KeyT, CompareT, loop_allocator<KeyT>>;

        template<typename KeyT, typename HashT = std::hash<KeyT>, typename KeyEqT = std::equal_to<KeyT>>
        using unordered_set = std::unordered_set<KeyT, HashT, KeyEqT, loop_allocator<KeyT>>;

        template<typename KeyT, typename CompareT = std::less<KeyT>>
        using sorted_vector = mira::sorted_vector<KeyT, CompareT, loop_allocator<KeyT>>;

        template<typename KeyT, typename CompareT = std::less<KeyT>>
        using unique_vector = mira::unique_vector<KeyT, CompareT, loop_allocator<KeyT>>;
    }
}