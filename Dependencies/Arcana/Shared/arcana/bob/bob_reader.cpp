// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "bob_reader.h"
#include <cereal/archives/portable_binary.hpp>  // for PortableBinaryInputAr...
#include <algorithm>                            // for min_element
#include <iostream>                             // for basic_istream
#include <limits>                               // for numeric_limits

namespace mira
{
    bob_reader::bob_reader(std::unique_ptr<bob_stream_file> file, std::unique_ptr<std::istream> stream)
        : m_file{ std::move(file) }
        , m_itrs{ m_file->get_iterators() }
        , m_stream{ std::move(stream) }
    {
        // move all the iterators to their first messages
        for (auto& itr : m_itrs)
        {
            read(nullptr, itr);

            if (itr.EntryIdx != itr.LastRead.EntryIdx)
            {
                advance(itr);
            }
        }
    }

    bool bob_reader::advance()
    {
        auto next = std::min_element(m_itrs.begin(),
                                     m_itrs.end(),
                                     [](const bob_stream_file::iterator& left, const bob_stream_file::iterator& right) {
                                         return left.LastRead.Timestamp < right.LastRead.Timestamp;
                                     });

        if (next->LastRead.Timestamp == std::numeric_limits<decltype(bob::entry_instance::Timestamp)>::max())
            return false;

        m_stream->seekg(next->Position);

        {
            cereal::PortableBinaryInputArchive archive{ *m_stream };

            if (!read(&archive, *next))
            {
                return false;
            }

            auto handler = m_handlers.find(next->EntryIdx);
            if (handler != m_handlers.end())
            {
                handler->second(next->LastRead.Timestamp, archive);
            }
        }

        advance(*next);

        return true;
    }

    void bob_reader::advance(bob_stream_file::iterator& itr)
    {
        bool readsucceeded = false;
        do
        {
            itr.Position += itr.LastRead.NextBlockOffset;
            readsucceeded = read(nullptr, itr);
        } while (readsucceeded && itr.EntryIdx != itr.LastRead.EntryIdx);

        if (!readsucceeded)
        {
            // we reached eof, so mark the iterator as complete
            itr.LastRead.Timestamp = std::numeric_limits<decltype(bob::entry_instance::Timestamp)>::max();
        }
    }

    bool bob_reader::read(cereal::PortableBinaryInputArchive* archive, bob_stream_file::iterator& itr)
    {
        try
        {
            if (archive == nullptr)
            {
                m_stream->seekg(itr.Position);

                cereal::PortableBinaryInputArchive ar{ *m_stream };

                itr.LastRead.serialize(ar, m_file->version());
            }
            else
            {
                itr.LastRead.serialize(*archive, m_file->version());
            }

            return true;
        }
        catch (const cereal::Exception&)
        {
            // we reached eof, we have to clear the state or else
            // all the next operations are going to fail.
            m_stream->clear();

            return false;
        }
    }
}
