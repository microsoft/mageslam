// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Types.h"

namespace mage
{
    template<typename OriginalT>
    class ProxyFactory
    {
    public:
        template<typename ProxyT, typename ...Values>
        static ProxyT CreateNew(Values&&... args)
        {
            return ProxyT{ s_generator.generate(), std::forward<Values>(args)... };
        }

    private:
        static IdGenerator<OriginalT> s_generator;
    };

    template<typename T>
    IdGenerator<T> ProxyFactory<T>::s_generator = {};
}