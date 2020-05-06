// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "arcana/bob/bob_helpers.h"

#include <cstdint>
#include <string>
#include <vector>

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

#include <gsl/gsl>

namespace mira
{
    namespace bob
    {
        namespace v1
        {
            namespace
            {
                constexpr std::uint32_t VERSION = 1;
            }

            //
            // manifest and stream_file_header are both types found in the json
            // manifest file. This file describes in a human readable format what the bob
            // stream contains and which data it depends on.
            //
            // entry_instance is the header data of each instance of an entry stream in the
            // binary stream file (.bob).
            //

            //
            //  Describes what the binary stream file contains
            //
            class stream_file_header
            {
            public:
                //
                //  An entry describes the properties of one type of packet found in the stream
                //
                class entry
                {
                public:
                    entry()
                        : m_name{}
                        , m_type{}
                        , m_version{}
                    {}

                    entry(std::string name, ::uint32_t version)
                        : m_name{ std::move(name) }
                        , m_type{}
                        , m_version{ version }
                    {}

                    entry(std::string name, std::string type, ::uint32_t version)
                        : m_name{ std::move(name) }
                        , m_type{ std::move(type) }
                        , m_version{ version }
                    {}

                    const std::string& name() const
                    {
                        return m_name;
                    }

                    const std::string& type() const
                    {
                        return m_type;
                    }

                    std::uint32_t version() const
                    {
                        return m_version;
                    }

                    template<typename Archive>
                    void serialize(Archive& ar)
                    {
                        ar(cereal::make_nvp("Name", m_name), cereal::make_nvp("Type", m_type), cereal::make_nvp("Version", m_version));
                    }

                private:
                    std::string m_name;
                    std::string m_type;
                    std::uint32_t m_version;
                };

                stream_file_header() = default;

                explicit stream_file_header(std::vector<entry> entries)
                    : m_entries(std::move(entries))
                {}

                std::uint32_t version() const
                {
                    return VERSION;
                }

                const std::vector<entry>& entries() const
                {
                    return m_entries;
                }

                template<typename Archive>
                void serialize(Archive& ar, const std::uint32_t version)
                {
                    if (version != VERSION)
                        throw_invalid_version();

                    ar(cereal::make_nvp("Entries", m_entries));
                }

            private:
                std::vector<entry> m_entries;
            };

            class manifest
            {
            public:
                manifest() = default;

                manifest(stream_file_header header)
                    : m_stream{ std::move(header) }
                {}

                std::uint32_t version() const
                {
                    return VERSION;
                }

                const stream_file_header& stream() const
                {
                    return m_stream;
                }

                template<typename Archive>
                void serialize(Archive& ar, const std::uint32_t version)
                {
                    if (version != VERSION)
                        throw_invalid_version();

                    ar(cereal::make_nvp("Stream", m_stream));
                }

            private:
                stream_file_header m_stream;
            };

            //
            // describes an entry in the binary stream, the data for the entry
            // is contained between this block and the next.
            //
            struct entry_instance
            {
                std::int64_t Timestamp;
                std::int64_t NextBlockOffset;
                std::int32_t EntryIdx;

                template<typename Archive>
                void serialize(Archive& ar, const std::uint32_t version)
                {
                    if (version != VERSION)
                        throw_invalid_version();

                    ar(CEREAL_NVP(Timestamp), CEREAL_NVP(NextBlockOffset), CEREAL_NVP(EntryIdx));
                }
            };
        }

        using namespace v1;
    }
}

CEREAL_CLASS_VERSION(::mira::bob::v1::manifest, ::mira::bob::v1::VERSION);
CEREAL_CLASS_VERSION(::mira::bob::v1::stream_file_header, ::mira::bob::v1::VERSION);
