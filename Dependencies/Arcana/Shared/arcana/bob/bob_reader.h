// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "bob_stream_file.h"           // for bob_stream_file, bob_stream_fi...
#include "arcana/bob/bob_data.h"       // for stream_file_header, stream_fil...
#include <cereal/cereal.hpp>           // for make_nvp, InputArchive
#include <cereal/archives/json.hpp>    // for JSONInputArchive
#include <cereal/details/helpers.hpp>  // for Exception
#include <assert.h>                    // for assert
#include <stdint.h>                    // for int64_t, uint32_t, int32_t
#include <functional>                  // for function
#include <iosfwd>                      // for istream
#include <list>                        // for _List_const_iterator
#include <memory>                      // for unique_ptr
#include <string>                      // for string
#include <unordered_map>               // for unordered_map, _Umap_traits<>:...
#include <vector>                      // for vector

namespace cereal { class PortableBinaryInputArchive; }  // lines 14-14

namespace mira
{
    class bob_reader
    {
    public:
        template<typename EntryT>
        using callback_t =
            std::function<void(const typename EntryT::context_t& context, const std::int64_t&, const EntryT&)>;

        bob_reader(std::unique_ptr<bob_stream_file> file, std::unique_ptr<std::istream> stream);

        template<typename EntryT>
        void handle(const std::string& streamName, callback_t<EntryT> callback)
        {
            auto stream = m_file->get_stream(streamName);
            m_handlers[stream.EntryIdx] = entry_reader<EntryT>{ *m_stream, std::move(callback), stream.Entry.version() };
        }

        bool contains_handler_for_stream(const std::string& streamName) const
        {
            if ( contains_stream(streamName) )
            {
                auto stream = m_file->get_stream(streamName);
                return m_handlers.find(stream.EntryIdx) != m_handlers.end();
            }

            return false;
        }

        bool contains_stream(const std::string& streamName) const
        {
            return m_file->contains_stream(streamName);
        }

        const std::vector<bob::stream_file_header::entry>& entries() const
        {
            return file().entries();
        }

        bool advance();

        const bob_stream_file& file() const
        {
            return *m_file;
        }

        bob_stream_file& file()
        {
            return *m_file;
        }

    private:
        void advance(bob_stream_file::iterator& itr);
        bool read(cereal::PortableBinaryInputArchive* archive, bob_stream_file::iterator& itr);

        template<typename EntryT>
        struct entry_reader
        {
            using context_t = typename EntryT::context_t;

            entry_reader(std::istream& stream, callback_t<EntryT> callback, std::uint32_t version)
                : m_stream{ stream }
                , m_callback{ std::move(callback) }
                , m_version{ version }
            {}

            bool operator()(const std::int64_t& timestamp, cereal::PortableBinaryInputArchive& archive)
            {
                if (!m_contextRead)
                {
                    m_contextRead = true;

                    cereal::JSONInputArchive json{ m_stream };

                    std::string name;
                    json(cereal::make_nvp("_Stream", name));

                    std::uint32_t version;
                    json(cereal::make_nvp("_Version", version));
                    assert(version == m_version);

                    m_context.serialize(json, m_version);

                    return true;
                }

                EntryT element;

                // we manage versioning in the header to avoid the overhead,
                // which means we need to call the serialize function with
                // the version number.
                try
                {
                    element.serialize(archive, m_version);
                }
                catch (cereal::Exception&)
                {
                    // we reached eof, we have to clear the state or else
                    // all the next operations are going to fail.
                    m_stream.clear();
                    return false;
                }

                m_callback(m_context, timestamp, element);

                return true;
            }

            std::istream& m_stream;
            callback_t<EntryT> m_callback;
            std::uint32_t m_version;

            // the streams context data
            context_t m_context;
            bool m_contextRead{ false };
        };

        std::unique_ptr<bob_stream_file> m_file;
        std::vector<bob_stream_file::iterator> m_itrs;
        std::unique_ptr<std::istream> m_stream;

        using entry_reader_func = std::function<bool(const std::int64_t&, cereal::PortableBinaryInputArchive&)>;

        std::unordered_map<std::int32_t, entry_reader_func> m_handlers;
    };
}
