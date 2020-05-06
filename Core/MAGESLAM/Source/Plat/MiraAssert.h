// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <Plat/FormatString/FormatString.h>

/*
    Usage:
        MIRA_ASSERT(expression);
        MIRA_ASSERT(expression, message);
        MIRA_FAIL(message);
*/

#ifdef _MSC_VER
#include <Windows.h>
#define MIRA_BREAKPOINT() __debugbreak()
#define MIRA_OUTPUTDEBUGSTRING(x) OutputDebugStringW(x)
#else
#define MIRA_BREAKPOINT()
#define MIRA_OUTPUTDEBUGSTRING(x)
#endif

#if defined(DEBUG) || defined(BUILD_DEV)
#define MIRA_ASSERT_ENABLED

// Use do-while to avoid incorrect logic happening if assert is used in an if-statement without braces.
#define MIRA_ASSERT2(condition, message) do { \
        if (!(condition)) \
        { \
            MIRA_OUTPUTDEBUGSTRING(L"\n"); \
            mira::LogError(mira::FormatString(L"%s\nFile: %s\nLine: %d\nExpression: %s", (message), __FILE__, __LINE__, L###condition).c_str()); \
            MIRA_OUTPUTDEBUGSTRING(L"\n"); \
            MIRA_BREAKPOINT(); \
        } \
    } while(0)
#else
#define MIRA_ASSERT2(condition, message)
#endif

#define MIRA_ASSERT1(condition) MIRA_ASSERT2(condition, L###condition)

// These macros allow MIRA_ASSERT to have one- and two-argument variations, making the message parameter optional.
// MIRA_ASSERT_ID works around the weird way Visual Studio implements __VA_ARGS__ http://stackoverflow.com/questions/25144589/c-macro-overloading-is-not-working
#define MIRA_ASSERT_ID(x) x
#define MIRA_ASSERT_NUMARGS_LOOKUP(_1, _2, MACRONAME, ...) MACRONAME
#define MIRA_ASSERT(...) MIRA_ASSERT_ID(MIRA_ASSERT_NUMARGS_LOOKUP(__VA_ARGS__, MIRA_ASSERT2, MIRA_ASSERT1)(__VA_ARGS__))

#define MIRA_FAIL(message) MIRA_ASSERT(false, message)

// Placeholder for contracts http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4415.pdf
#define MIRA_EXPECTS(precondition) MIRA_ASSERT(precondition, L"Precondition failure")
#define MIRA_ENSURES(postcondition) MIRA_ASSERT(postcondition, L"Postcondition failure")
