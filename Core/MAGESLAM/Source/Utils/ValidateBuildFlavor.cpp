// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "ValidateBuildFlavor.h"
#include <stdexcept>
#include <assert.h>
#include <iostream>

namespace mira
{
#pragma warning ( push )
#pragma warning ( disable : 4706 )
    void ValidateBuildFlavor()
    {
        //#error "This tool not intended to use production build flavor, reset to dev flavor"
        // instead we'll trap the debugger (to help developers understand what is wrong) and exit the program
        bool assertsEnabled = false;
        assert(assertsEnabled = true);
        if (!assertsEnabled)
        {
            std::cerr << "build configuration invalid, expect that asserts are enabled" << std::endl;
            throw std::logic_error("build configuration invalid, expect that asserts are enabled");
        }
    }
#pragma warning ( pop )
}
