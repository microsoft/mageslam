// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "introspector.h"            // for introspector

#include <cereal/archives/json.hpp>  // for JSONOutputArchive

#include <fstream>
#include <functional>                // for function, _Func_class
#include <memory>                    // for unique_ptr, make_unique
#include <mutex>                     // for mutex, lock_guard
#include <string>                    // for operator+, basic_string, string
#include <type_traits>               // for forward
#include <vector>                    // for vector

namespace mira
{
    //
    //  A helper class for getting very detailed information
    // on all the input and output parameter values of a function.
    //
    //  It is responsible for introspecting all the input and output
    // parameters of a function and saving the results into a file
    // as json for further analysis.
    //
    //  Usage:
    //  void foo(int one, int two, int& out)
    //  {
    //      XRAY_FUNCTION(
    //          "scope",
    //          XR_INPUT(one, two)
    //          XR_OUTPUT(out)
    //      );
    //      out = one + two;
    //  }
    //
    //  The scope determines in what file the xray will be saved to,
    // in this case it will be located in your current directory under the
    // name "scope.xray.json"
    //
    class xray
    {
    public:
        //
        // func: the __FUNCTION__ name you are introspecting
        // scope: The scope of the xray. If you're xray'ing
        //      all the invocations in a singleton_foo you'd
        //      probably want the scope to be singleton_foo.
        //      The file in which the xray is saved is derived
        //      from the scope name.
        //
        xray(const char* func, const char* scope)
            : m_scope{ scope }
        {
            m_archive->callable().setNextName(func);
            m_archive->callable().startNode();
        }

        // constructor to allow passing in a stringstream for testing purposes
        xray(const char* func, const char* scope, std::stringstream& ss)
            : m_archive{ std::make_unique<introspector<cereal::JSONOutputArchive>>(ss) }
            , m_scope{ scope }                
        {
            m_archive->callable().setNextName(func);
            m_archive->callable().startNode();
        }

        ~xray()
        {
            write_output();

            m_archive->callable().finishNode();

            // clear out the archive so it closes the last object.
            m_archive = nullptr;

            // don't flush to file if we're writing to a test
            // stream instead of our inplace stream.
            if (m_ss.tellp() > 0)
            {
                m_ss << "\n";

                auto string = m_ss.str();

                {
                    std::lock_guard<std::mutex> guard{ m_mutex };

                    std::ofstream stream{ m_scope + ".xray.json", std::ios::app };

                    stream.write(string.c_str(), string.length());
                }
            }
        }

        template<typename ...ArgTs>
        xray& input(ArgTs&& ...args)
        {
            m_archive->callable().setNextName("Input");
            m_archive->callable().startNode();

            (*m_archive)(std::forward<ArgTs>(args)...);

            m_archive->callable().finishNode();

            return *this;
        }

        template<typename FuncT>
        xray& delayed_output(FuncT&& argFactory)
        {
            m_output.emplace_back(std::forward<FuncT>(argFactory));

            return *this;
        }

        template<typename ...ArgTs>
        void output_args(ArgTs&& ...args)
        {
            (*m_archive)(std::forward<ArgTs>(args)...);
        }

    private:
        void write_output()
        {
            m_archive->callable().setNextName("Output");
            m_archive->callable().startNode();

            for (auto& func : m_output)
                func();

            m_archive->callable().finishNode();

            m_output.clear();
        }

        std::stringstream m_ss{};
        std::unique_ptr<introspector<cereal::JSONOutputArchive>> m_archive = std::make_unique<introspector<cereal::JSONOutputArchive>>(m_ss);

        std::vector<std::function<void()>> m_output{};
        const std::string m_scope;

        static std::mutex m_mutex;
    };
}

#ifdef XRAY_ENABLED

#define XRAY_FUNCTION(SCOPE, INPUT_OUTPUT_MACROS, ...) ::mira::xray internal_function_xray( __FUNCTION__, SCOPE, __VA_ARGS__); internal_function_xray INPUT_OUTPUT_MACROS
#define XR_INPUT(...) .input(__VA_ARGS__)
#define XR_OUTPUT(...) .delayed_output([&] { internal_function_xray.output_args(__VA_ARGS__); })

#else

#define XRAY_FUNCTION(SCOPE, INPUT_OUTPUT_MACROS) 
#define XR_INPUT(...)
#define XR_OUTPUT(...)

#endif
