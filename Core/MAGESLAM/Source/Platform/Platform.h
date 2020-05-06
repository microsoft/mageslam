// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mage
{
    namespace platform
    {
        // TODO: Stub void set_thread_name(const char* name);
        inline void set_thread_name(const char*) {};
        void profile_memory();

        void wait_for_debugger();
    }
}
