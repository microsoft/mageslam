// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <stdint.h>

#include "arcana\utils\algorithm.h"
#include "std_allocator.h"

namespace mage
{
    namespace memory
    {
        struct factory
        {
            template<typename T, typename ...Args>
            static void construct(T* p, Args&&... args)
            {
                ::new (static_cast<void*>(p)) T(std::forward<Args>(args)...);
            }

#pragma warning(push)
#pragma warning(disable: 4100)
            /*
            For some reason when a type doesn't have a constructor defined
            the optimizer removes the destruction call line, but then emits
            the "p is unused warning" which then becomes a compiler error.
            */
            template<typename T>
            static void destroy(T* p)
            {
                p->~T();
            }
#pragma warning(pop)
        };

        /*
            Helper struct for defining the std compatible std::allocator_traits for our allocators
        */
        template<typename AllocT>
        struct allocator_traits
        {
            using allocator_type = AllocT;
            using value_type = typename AllocT::value_type;
            using pointer = value_type*;
            using const_pointer = const value_type*;
            using void_pointer = void*;
            using const_void_pointer = const void*;
            using difference_type = ptrdiff_t;
            using size_type = size_t;

            using propagate_on_container_copy_assignment = std::true_type;
            using propagate_on_container_move_assignment = std::true_type;
            using propagate_on_container_swap = std::true_type;

            using is_always_equal = std::false_type;

            template<typename D>
            using rebind_alloc = typename AllocT::template rebind<D>;

            static pointer allocate(allocator_type& a, size_t n)
            {
                return a.allocate(n);
            }

            static void deallocate(allocator_type& a, pointer p, size_t n)
            {
                a.deallocate(p, n);
            }

            static pointer allocate(allocator_type& a, size_t n, const void*)
            {
                return allocate(a, n);
            }

            /*
                Based on the docs if an object has a constructor,
                call it explicitely, if not fall back to placement new.
            */
            template<typename T, typename ...Args>
            static void construct(allocator_type&, T* p, Args&&... args)
            {
                factory::construct(p, std::forward<Args>(args)...);
            }

            /*
                For some reason when a type doesn't have a constructor defined
                the optimizer removes the destruction call line, but then emits
                the "p is unused warning" which then becomes a compiler error.
            */
            template<typename T>
            static void destroy(allocator_type&, T* p)
            {
                factory::destroy(p);
            }

            /*
                These are just stubs to make the stl happy, we return null
                when we can't allocate anyway
            */
            static size_t max_size(const allocator_type&)
            {
                return std::numeric_limits<size_t>::max() / sizeof(value_type);
            }

            static allocator_type select_on_container_copy_construction(const allocator_type& a)
            {
                return a;
            }

            /*
                hexagon libc++ specific definitions 
            */
            template<typename T>
            static void __construct_backward(allocator_type& alloc, T* __begin1, T* __end1, T* __end2)
            {
                while (__end1 != __begin1)
                {
                    construct(alloc, __end2-1, std::move_if_noexcept(*--__end1));
                    --__end2;
                }
            }

            template<typename T>
            static void __construct_forward(allocator_type& alloc, T* __begin1, T* __end1, T* __begin2)
            {
                for (; __begin1 != __end1; ++__begin1, ++__begin2)
                    construct(alloc, __begin2, std::move_if_noexcept(*__begin1));
            }

            template <typename IterT, typename T>
            static void __construct_range_forward(allocator_type& alloc, IterT __begin1, IterT __end1, T* __begin2)
            {
                for (; __begin1 != __end1; ++__begin1, ++__begin2)
                    construct(alloc, __begin2, *__begin1);
            }
        };
    }
}