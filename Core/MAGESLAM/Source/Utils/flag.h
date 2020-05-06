// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mage
{
    template<typename T>
    struct flag
    {
        flag(const T& value)
            : m_value{ value }
        {}
        
        /*
            Sets the value of the flag and returns its previous value.
        */
        T test_and_set(const T& value)
        {
            T value = std::move(m_value);
            m_value = value;
            return value;
        }

        /*
            Sets the value of the flag and returns its previous value.
        */
        T test_and_set(T&& value)
        {
            T oldValue = std::move(m_value);
            m_value = value;
            return oldValue;
        }

    private:
        T m_value;
    };
}