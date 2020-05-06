// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    // The state_machine_state represents work that can't run at the same time
    // as any other state_machine_state. It will also respect the dependencies
    // and order defined by the schedule the scheduler is running.
    // You need to define your scopes globally, they don't store any
    // data they just serve to identify the scopes that need to run.
    class abstract_state_machine_state
    {
    public:
        abstract_state_machine_state(const char* name)
            : m_name{ name }
        {}

        const char* name() const
        {
            return m_name;
        }

    private:
        abstract_state_machine_state(const abstract_state_machine_state&) = delete;
        abstract_state_machine_state& operator=(const abstract_state_machine_state&) = delete;

        const char* const m_name;
    };

    template<typename ResultT>
    class state_machine_state : public abstract_state_machine_state
    {
    public:
        using result_t = ResultT;

        state_machine_state(const char* name)
            : abstract_state_machine_state(name)
        {}
    };
}
