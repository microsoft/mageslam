// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    ///<summary>
    /// A class for passing callable objects as arguments to functions without allocation. 
    /// This should be used similar to a string_view. This does not 
    /// take ownership of the callable and will be invalid if the callable passed goes 
    /// out of scope before the function_iew that wraps it. 
    ///</summary>
    template<typename Fn>
    class function_view;

    template<typename Ret, typename... Params>
    class function_view<Ret(Params...)>
    {
    public:
        template <typename T>
        function_view(T&& callable)
            : m_invoker{ Invoke<typename std::remove_reference<T>::type> }
            , m_callable{ reinterpret_cast<void*>(&callable) }
        {
        }

        Ret operator()(Params ...params) const
        {
            return m_invoker(m_callable, std::forward<Params>(params)...);
        }

    private:
        template<typename T>
        static Ret Invoke(void* callable, Params ...params)
        {
            return (*reinterpret_cast<T*>(callable))(std::forward<Params>(params)...);
        }

        using InvokerPtr = Ret(*)(void*, Params...);

        InvokerPtr m_invoker;
        void* m_callable;
    };
}
