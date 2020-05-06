// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Platform.h"

#include "Utils\Logging.h"

#ifdef WINAPI_FAMILY_APP

#include <thread>
#include <vector>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <Windows.h>

using namespace std;

namespace mage
{
    namespace platform
    {
        namespace
        {
            //
            // Usage: SetThreadName ((DWORD)-1, "MainThread");
            // https://msdn.microsoft.com/en-us/library/xcb2z8hs.aspx
            const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push,8)
            typedef struct tagTHREADNAME_INFO
            {
                DWORD dwType; // Must be 0x1000.
                LPCSTR szName; // Pointer to name (in user addr space).
                DWORD dwThreadID; // Thread ID (-1=caller thread).
                DWORD dwFlags; // Reserved for future use, must be zero.
            } THREADNAME_INFO;
#pragma pack(pop)

            void SetThreadName(DWORD dwThreadID, const char* threadName)
            {
                THREADNAME_INFO info;
                info.dwType = 0x1000;
                info.szName = threadName;
                info.dwThreadID = dwThreadID;
                info.dwFlags = 0;
#pragma warning(push)
#pragma warning(disable: 6320 6322)
                __try {
                    RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
                }
                __except (EXCEPTION_EXECUTE_HANDLER) {
                }
#pragma warning(pop)
            }
        }
        
        void set_thread_name(const char* name)
        {
            auto id = std::this_thread::get_id();
            SetThreadName(reinterpret_cast<DWORD&>(id), name);
        }

        int MAGEAllocHook(int nAllocType, void* pvData,
            size_t nSize, int /*nBlockUse*/, long /*lRequest*/,
            const unsigned char* /*szFileName*/, int /*nLine*/)
        {
            switch (nAllocType)
            {
            case _HOOK_ALLOC:
                LogStatistic(L"MemAlloc", L"CrtAlloc", nSize, reinterpret_cast<uint64_t>(pvData));
                break;
            case _HOOK_REALLOC:
                LogStatistic(L"MemAlloc", L"CrtReAlloc", nSize, reinterpret_cast<uint64_t>(pvData));
                break;
            case _HOOK_FREE:
                LogStatistic(L"MemAlloc", L"CrtFree", nSize, reinterpret_cast<uint64_t>(pvData));
                break;
            default:
                LogStatistic(L"MemAlloc", L"CrtUnknown", nSize, reinterpret_cast<uint64_t>(pvData));
            }
            return TRUE;
        }

        void profile_memory()
        {
            _CrtSetAllocHook(MAGEAllocHook);
        }

        void wait_for_debugger()
        {
            while (!IsDebuggerPresent())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds{ 30 });
            }

            OutputDebugStringA("\nWelcome to hell!\n");
            __debugbreak();
        }
    }
}

#endif
