// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "binary_iterator.h"  // for make_binary_iterator
#include "crc32.h"
#include <arcana/macros.h>    // for LOCATION_TAG
#include <cstddef>            // for size_t
#include <map>
#include <mutex>              // for mutex, lock_guard
#include <string>             // for string
#include <utility>            // for move
#include <vector>             // for vector

#if NDEBUG
#define DETERMINISTIC_CHECK(determinator, ...) (void)determinator
#else
#define DETERMINISTIC_CHECK(determinator, ...)                                                                         \
    determinator.iterator(LOCATION_TAG " -> " #__VA_ARGS__).iterate(__VA_ARGS__)
#endif

namespace mira
{
    class determinator
    {
    public:
        auto iterator(const char* tag)
        {
            return make_binary_iterator(iterator_callback{ tag, *this });
        }

        void set_enabled(bool value)
        {
            m_enabled = value;
        }

        template<class Archive>
        void save(Archive& archive) const
        {
            archive(m_samples);
        }

        template<class Archive>
        void load(Archive& archive)
        {
            archive(m_truth);
        }

        static determinator& create(const std::string& name)
        {
            std::lock_guard<std::mutex> guard{ m_detlock };
            auto& el = m_determinators[name];
            el.m_samples.clear();
            return el;
        }

        static determinator& fetch(const std::string& name)
        {
            std::lock_guard<std::mutex> guard{ m_detlock };
            return m_determinators[name];
        }

        class iterator_callback
        {
        public:
            iterator_callback(const char* tag, determinator& det)
                : m_tag{ tag }
                , m_det{ &det }
            {}

            iterator_callback(iterator_callback&& other)
                : m_tag{ other.m_tag }
                , m_det{ other.m_det }
                , m_crc{ std::move(other.m_crc) }
            {
                other.m_tag = nullptr;
                other.m_det = nullptr;
            }

            ~iterator_callback()
            {
                if (m_det)
                {
                    m_det->process_crc(m_crc.checksum());
                }
            }

            void operator()(const void* data, size_t bytes)
            {
                m_crc.process(data, bytes);
            }

        private:
            const char* m_tag;
            determinator* m_det;
            continuous_crc32 m_crc;
        };

    private:
        friend class binary_iterator<iterator_callback>;

        void process_crc(size_t value);

        std::vector<size_t> m_samples;
        std::vector<size_t> m_truth;
        bool m_enabled{ false };

        static std::mutex m_detlock;
        static std::map<std::string, determinator> m_determinators;
    };
}
