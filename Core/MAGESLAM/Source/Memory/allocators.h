// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "allocator_traits.h"
#include "std_allocator.h"
#include "stack_allocator.h"
#include "mallocator.h"
#include "debug_electric_fence.h"
#include "debug_check_fail.h"
#include "debug_tracking_allocator.h"
#include "debug_memory_marking_allocator.h"
#include "non_deallocating_allocator.h"
#include "fallback_allocator.h"
#include "free_list.h"

namespace mage
{
    using stack_allocation_strategy =
        memory::debug_electric_fence<
            memory::debug_check_fail<
                memory::debug_memory_marking_allocator<
                    memory::non_deallocating_allocator<
                        memory::stack_allocator<
                            memory::mallocator
        >>>>>;

    using loop_allocation_strategy =
        memory::debug_electric_fence<
            memory::debug_check_fail<
                memory::debug_memory_marking_allocator<
                    memory::non_deallocating_allocator<
                        memory::stack_allocator<
                            memory::mallocator
        >>>>>;

    using image_allocation_strategy = 
        memory::debug_check_fail<
            memory::debug_tracking_allocator<
                memory::fallback_allocator<
                    memory::debug_memory_marking_allocator<
                        memory::free_list<
                            memory::stack_allocator<
                                memory::mallocator
                    >>>,
                    memory::mallocator>>>;

    using block_splitting_allocation_strategy = memory::non_owning_stack_allocator<alignof(int32_t)>;

    namespace memory
    {
        namespace tags
        {
            struct loop {};
            struct stack {};
        }
    }

    template<typename T>
    using stack_allocator = memory::std_allocator<T, stack_allocation_strategy, memory::tags::stack>;

    template<typename T>
    using loop_allocator = memory::std_allocator<T, loop_allocation_strategy, memory::tags::loop>;
}

namespace std
{
    /*
        The allocator traits that make our allocators function in regular stl
        containers.
    */
    template<typename T>
    struct allocator_traits<mage::stack_allocator<T>> : public mage::memory::allocator_traits<mage::stack_allocator<T>>
    {};

    template<typename T>
    struct allocator_traits<mage::loop_allocator<T>> : public mage::memory::allocator_traits<mage::loop_allocator<T>>
    {};
}