// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Memory\allocation_scope.h"
#include "Memory\allocators.h"
#include "Containers\GenericContainers.h"

namespace mage
{
    struct thread_memory
    {
        using stack_scope_type = memory::allocation_scope<stack_allocation_strategy>;
        using loop_scope_type = memory::allocation_scope<loop_allocation_strategy>;

        thread_memory(loop_scope_type& loopAllocator, stack_scope_type stackAllocator)
            : m_loopAllocator{ loopAllocator }, m_stackAllocator{ stackAllocator }
        {}

        template<typename T>
        temp::vector<T> stack_vector(size_t reserve)
        {
            temp::vector<T> vec{ stack_allocator<T>{ m_stackAllocator.allocator() } };
            vec.reserve(reserve);
            return vec;
        }
        
        template<typename T>
        temp::vector<T> stack_vector()
        {
            return temp::vector<T>{ stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        temp::set<T, CompareT> stack_set()
        {
            return temp::set<T, CompareT>{ stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T, typename HashT = std::hash<T>>
        temp::unordered_set<T, HashT> stack_unordered_set()
        {
            return temp::unordered_set<T, HashT>{ stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename KeyT, typename ValT, typename HashT = std::hash<KeyT>>
        temp::unordered_map<KeyT, ValT, HashT> stack_unordered_map()
        {
            return temp::unordered_map<KeyT, ValT, HashT>{ stack_allocator<std::pair<const KeyT, ValT>>{ m_stackAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        temp::unique_vector<T, CompareT> stack_unique_vector()
        {
            return temp::unique_vector<T, CompareT>{ stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        temp::unique_vector<T, CompareT> stack_unique_vector(size_t reserved)
        {
            return temp::unique_vector<T, CompareT>{ reserved, stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        temp::sorted_vector<T, CompareT> stack_sorted_vector()
        {
            return temp::sorted_vector<T, CompareT>{ stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        temp::sorted_vector<T, CompareT> stack_sorted_vector(size_t reserved)
        {
            return temp::sorted_vector<T, CompareT>{ reserved, stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T>
        temp::buffer<T> stack_buffer(size_t size)
        {
            return{ size, stack_allocator<T>{ m_stackAllocator.allocator() } };
        }

        template<typename T>
        loop::vector<T> loop_vector(size_t reserve)
        {
            loop::vector<T> vec{ loop_allocator<T>{ m_loopAllocator.allocator() } };
            vec.reserve(reserve);
            return vec;
        }

        template<typename T>
        loop::vector<T> loop_vector()
        {
            loop::vector<T> vec{ loop_allocator<T>{ m_loopAllocator.allocator() } };
            return vec;
        }

        template<typename T>
        loop::set<T> loop_set()
        {
            return loop::set<T>{ loop_allocator<T>{ m_loopAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        loop::unique_vector<T, CompareT> loop_unique_vector()
        {
            return loop::unique_vector<T, CompareT>{ loop_allocator<T>{ m_loopAllocator.allocator() } };
        }

        template<typename T, typename CompareT = std::less<T>>
        loop::unique_vector<T, CompareT> loop_unique_vector(size_t reserved)
        {
            return loop::unique_vector<T, CompareT>{ reserved, loop_allocator<T>{ m_loopAllocator.allocator() } };
        }
    private:
        /*
            the allocator that deallocates every iteration of the thread loop,
            this is passed down by ref because we want to use the same allocation
            scope for the entire loop.
        */
        loop_scope_type& m_loopAllocator;

        /*
            the allocator that deallocates at the end of each function call
            for temporary data. It relies on copy semantics to create a new scope
            when passed to a function, and destroy the scope when returning from
            the function as the parameter gets destroyed.
        */
        stack_scope_type m_stackAllocator;
    };

    /*
        Helper struct for easily defining temporary memory when not
        in a thread.
    */
    struct temp_memory
    {
        temp_memory(size_t loopSize = 100 * 1024, size_t stackSize = 100 * 1024)
            : loop{ loopSize }, stack{ stackSize }
        {}

        operator thread_memory()
        {
            return{ loopscope, thread_memory::stack_scope_type{ stack } };
        }

    private:
        // Create a temporary memory buffer for calling the graph updates
        loop_allocation_strategy loop;
        stack_allocation_strategy stack;
        thread_memory::loop_scope_type loopscope{ loop };
    };

    /*
        Helper struct for keeping memory around and converting it to thread_memory.
        Every time an instance of thread_memory is created it will reset the scopes.
    */
    struct memory_pool
    {
        memory_pool(size_t loopSize = 100 * 1024, size_t stackSize = 100 * 1024)
            : loop{ loopSize }, stack{ stackSize }
        {}

        //
        // clears the current loop scope and returns a new thread_memory instance
        //
        thread_memory create()
        {
            // we need to clear the previous instance
            // before we create a new one with the same loop memory.
            loopscope.reset();

            loopscope = std::make_unique<thread_memory::loop_scope_type>(loop);
            return { *loopscope, thread_memory::stack_scope_type{ stack } };
        }

        //
        // keeps the loop memory scope alive and returns a new thread_memory instance
        //
        thread_memory reuse()
        {
            return { *loopscope, thread_memory::stack_scope_type{ stack } };
        }
    private:
        // Create a temporary memory buffer for calling the graph updates
        loop_allocation_strategy loop;
        stack_allocation_strategy stack;
        std::unique_ptr<thread_memory::loop_scope_type> loopscope;
    };
}
