// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace mira
{
    template<typename T>
    std::weak_ptr<T> make_weak(const std::shared_ptr<T>& ptr)
    {
        return ptr;
    }
}
