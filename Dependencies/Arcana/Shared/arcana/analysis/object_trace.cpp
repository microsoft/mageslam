// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "object_trace.h"

namespace mira
{

    std::recursive_mutex object_trace::m_mutex{};
    std::unordered_map<std::type_index, object_trace::listeners> object_trace::m_typemap{};

    object_trace::channel_ticket object_trace::add_listener(
        const char* channel, const std::type_index& type, channel_callback callback)
    {
        std::lock_guard<std::recursive_mutex> guard{ m_mutex };

        return m_typemap[type].ChannelListeners[channel].insert(std::move(callback), m_mutex);
    }

    object_trace::global_ticket object_trace::add_listener(
        const std::type_index& type, global_callback callback)
    {
        std::lock_guard<std::recursive_mutex> guard{ m_mutex };

        return m_typemap[type].GlobalListeners.insert(std::move(callback), m_mutex);
    }

    void object_trace::fire_event(const char* channelName, std::intptr_t instance, const std::type_index& type, const void* data)
    {
        std::lock_guard<std::recursive_mutex> guard{ m_mutex };

        auto channels = m_typemap.find(type);
        if (channels == m_typemap.end())
            return;

        auto channelListeners = channels->second.ChannelListeners.find(gsl::cstring_span<>{ gsl::ensure_z(channelName) });
        if (channelListeners != channels->second.ChannelListeners.end())
        {
            for (auto& listener : channelListeners->second)
            {
                listener(instance, data);
            }
        }

        for (auto& listener : channels->second.GlobalListeners)
        {
            listener(channelName, instance, data);
        }
    }
}
