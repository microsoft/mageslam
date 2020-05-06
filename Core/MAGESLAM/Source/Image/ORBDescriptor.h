// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <opencv2\core.hpp>

#include "arcana\utils\Serialization\serializable.h"

namespace mage
{
    struct ORBDescriptor : mira::serializable<ORBDescriptor>
    {
        static constexpr int COLUMNS = 32;
        static constexpr int ROWS = 1;
        static constexpr int CV_TYPE = CV_8U;

        using ColumnT = uint8_t;

        static constexpr int BITS_PER_COLUMN = sizeof(ColumnT) * CHAR_BIT;
        static constexpr size_t DESCRIPTOR_SIZE_BYTES = sizeof(ColumnT) * COLUMNS * ROWS;

        /*
            Use cv::Mat for references to ORBDescriptors for now
            as sometimes we want to create new descriptors and other
            times we want to just refer to the data. Because cv::Mats
            already handle the owning semantics, as a first stop this
            remains the easiest.
        */
        struct Ref
        {
            const ORBDescriptor* operator->() const
            {
                return m_desc;
            }

            const ORBDescriptor& operator*() const
            {
                return *m_desc;
            }

            explicit Ref(const ORBDescriptor& desc)
                : m_desc{ &desc }
            {}
        private:

            friend struct ORBDescriptor;

            const ORBDescriptor* m_desc;
        };


        explicit ORBDescriptor(const cv::Mat& mat)
        {
            CopyFrom(mat);
        }

        explicit ORBDescriptor()
        {
            memset(m_desc, 0, sizeof(ColumnT) * COLUMNS);
        }

        ORBDescriptor(const ORBDescriptor&) = default;
        ORBDescriptor& operator=(const ORBDescriptor&) = default;

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        ORBDescriptor(StreamT& stream)
        {
            deserialize(stream);
        }

        static constexpr auto members()
        {
            return declare_members(
                &ORBDescriptor::m_desc
            );
        }

        ColumnT* Data()
        {
            return m_desc;
        }

        const ColumnT* Data() const
        {
            return m_desc;
        }

        void CopyTo(cv::Mat mat) const
        {
            assert(mat.size().area() == COLUMNS);
            
            memcpy(mat.ptr(), m_desc, sizeof(m_desc));
        }

        void CopyTo(ORBDescriptor& desc) const
        {
            memcpy(desc.m_desc, m_desc, sizeof(m_desc));
        }

        void CopyFrom(const cv::Mat& mat)
        {
            assert(mat.size().area() == COLUMNS);

            memcpy(m_desc, mat.ptr(), mat.size().area());
        }

        void CopyFrom(const ORBDescriptor& desc)
        {
            memcpy(m_desc, desc.m_desc, sizeof(desc.m_desc));
        }

        Ref AsRef() const
        {
            return Ref{ *this };
        }

        cv::Mat AsMat()
        {
            return cv::Mat{ ROWS, COLUMNS, CV_TYPE, (void*)m_desc, cv::Mat::CONTINUOUS_FLAG };
        }

        /*
            cv::Mats don't know what const means so for now just cast to void
            and ignore constness until we remove the use of cv::Mats entirely
        */
        cv::Mat AsMat() const
        {
            return cv::Mat{ ROWS, COLUMNS, CV_TYPE, (void*)m_desc, cv::Mat::CONTINUOUS_FLAG };
        }

    private:
        ColumnT m_desc[COLUMNS];
    };
}
