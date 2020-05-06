// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <atomic>
#include <chrono>

namespace mira
{
    /*
        templated class to give the ability to types to define
        how they should be serialized. Specialize this class in order
        determine what gets saved and loaded to the stream.
    */
    template<typename T>
    struct custom_serialization
    {
        template<typename StreamT>
        static void read(StreamT& stream, T& value)
        {
            stream.read(value);
        }

        template<typename StreamT>
        static void write(StreamT& stream, const T& value)
        {
            stream.write(value);
        }
    };

    template<typename T>
    struct custom_serialization<std::atomic<T>>
    {
        template<typename StreamT>
        static void read(StreamT& stream, std::atomic<T>& value)
        {
            T val;
            stream.read(val);
            value.store(std::move(val));
        }

        template<typename StreamT>
        static void write(StreamT& stream, const std::atomic<T>& value)
        {
            stream.write(value.load());
        }
    };

    template<typename Clock, typename Duration>
    struct custom_serialization<std::chrono::time_point<Clock, Duration>>
    {
        using time_point = std::chrono::time_point<Clock, Duration>;

        template<typename StreamT>
        static void read(StreamT& stream, time_point& value)
        {
            typename time_point::duration::rep r;
            stream.read(r);
            value = time_point{ typename time_point::duration{ r } };
        }

        template<typename StreamT>
        static void write(StreamT& stream, const time_point& value)
        {
            stream.write(value.time_since_epoch().count());
        }
    };

    template<typename T, size_t N>
    struct custom_serialization<T[N]>
    {
        template<typename StreamT>
        static void read(StreamT& stream, T values[N])
        {
            stream.read(reinterpret_cast<char*>(values), N * sizeof(T));
        }

        template<typename StreamT>
        static void write(StreamT& stream, const T values[N])
        {
            stream.write(reinterpret_cast<const char*>(values), N * sizeof(T));
        }
    };

    template<size_t N, typename StreamT, typename T>
    inline void write_multiple(StreamT& stream, const T* values)
    {
        // Write out {} to help with grouping when
        // reading the file, it isn't really necessary
        // it's just to help easily analyze a file after
        // the fact to notice issues
        stream.write('{');
        for (size_t i = 0; i < N - 1; ++i, ++values)
        {
            stream.write(*values);
        }
        stream.write(*values);
        stream.write('}');
    }

    template<size_t N, typename StreamT, typename T>
    inline void read_multiple(StreamT& stream, T* values)
    {
        char unused;
        stream.read(unused); // '{'
        for (size_t i = 0; i < N - 1; ++i, ++values)
        {
            stream.read(*values);
        }
        stream.read(*values);
        stream.read(unused); // '}'
    }
}