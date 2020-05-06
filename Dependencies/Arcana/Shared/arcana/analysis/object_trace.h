// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <unordered_map>
#include <mutex>
#include <gsl/gsl>

#include <typeindex>

#include "arcana/containers/ticketed_collection.h"
#include "arcana/string.h"

#if NDEBUG
#define FIRE_OBJECT_TRACE(...)
#else
#define OBJECT_TRACE_ENABLED
#define FIRE_OBJECT_TRACE(channelName, instance, evt) \
    ::mira::object_trace::fire(channelName, instance, evt)
#endif

namespace mira
{
    /*
        Simple static analysis of data for exposing
    */
    class object_trace final
    {
        using channel_callback = std::function<void(std::intptr_t, const void*)>;
        using global_callback = std::function<void(const char*, std::intptr_t, const void*)>;

        using channel_ticketed_collection = ticketed_collection<channel_callback, std::recursive_mutex>;
        using global_ticketed_collection = ticketed_collection<global_callback, std::recursive_mutex>;
    public:
        using channel_ticket = channel_ticketed_collection::ticket;
        using global_ticket = global_ticketed_collection::ticket;

        object_trace() = delete;

        /*
        Adds a listener to type T on the specified channel.

        The channel is useful to split data into logical streams.
        */
        template<typename T>
        static auto listen(const char* channel, std::function<void(std::intptr_t, T&)> callback)
        {
            return add_listener(channel, std::type_index{ typeid(std::decay_t<T>) }, [callback = std::move(callback)](std::intptr_t instance, const void* data)
            {
                callback(instance, const_cast<T&>(*reinterpret_cast<const T*>(data)));
            });
        }

        /*
            Adds a listener to type T to all channels.
        */
        template<typename T>
        static auto listen(std::function<void(const char*, std::intptr_t, T&)> callback)
        {
            return add_listener(std::type_index{ typeid(std::decay_t<T>) },
                [callback = std::move(callback)](const char* channel, std::intptr_t instance, const void* data)
                {
                    callback(channel, instance, const_cast<T&>(*reinterpret_cast<const T*>(data)));
                });
        }

        /*
            Fires an event in the specified channel.
        */
        template<typename T>
        static void fire(const char* channel, const void* instance, const T& evt)
        {
            fire_event(channel, (std::intptr_t)instance, std::type_index{ typeid(std::decay_t<T>) }, &evt);
        }
    private:
        using channels = std::map<std::string, channel_ticketed_collection, string_compare>;

        struct listeners
        {
            channels ChannelListeners;
            global_ticketed_collection GlobalListeners;
        };

        static channel_ticket add_listener(const char* channel, const std::type_index& type, channel_callback callback);
        static global_ticket add_listener(const std::type_index& type, global_callback callback);

        static void fire_event(const char* channel, std::intptr_t instance, const std::type_index& type, const void* data);

        static std::recursive_mutex m_mutex;
        static std::unordered_map<std::type_index, listeners> m_typemap;
    };
}
