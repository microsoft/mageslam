// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#define CONCATENATE_MACRO(x, y) CONCATENATE_MACRO_IMPL(x, y)
#define CONCATENATE_MACRO_IMPL(x, y) x##y

#define STRINGIFY_MACRO(s) STRINGIFY_MACRO_IMPL(s)
#define STRINGIFY_MACRO_IMPL(s) #s

#define WSTRINGIFY_MACRO(s) WSTRINGIFY_MACRO_IMPL(s)
#define WSTRINGIFY_MACRO_IMPL(s) L#s

#define LOCATION_TAG __FILE__ " Line: " STRINGIFY_MACRO(__LINE__)

// Defines a variable is unused to suppress warning 4100
// Intended for a variable used only in an assert which is compiled out in release
#define UNUSED(x) (void)(x)
