// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Utils\buffer.h"
#include "MageSettings.h"           // for CameraIdentity
#include "ORBDescriptor.h"

#include "Memory\allocators.h"
#include "arcana\utils\serialization\serializable.h"

#include <opencv2\core\core.hpp>
#include <stdint.h>
#include <atomic>

#include <gsl\span>

namespace mage
{
    using ImageAllocator = memory::std_allocator<void*, block_splitting_allocation_strategy>;

    template<typename AllocT>
    struct ImageData : mira::serializable<ImageData<AllocT>>
    {
        ImageData(CameraIdentity cameraIdentity, size_t maxFeatures, float pyramidScale, size_t numLevels, float imageBorder, const AllocT& allocator)
            :   m_cameraIdentity{ cameraIdentity },
                m_featureCount{ 0 },
                m_pyramidScale { pyramidScale },
                m_numLevels { numLevels },
                m_imageBorder { imageBorder },
                m_published{ false },
                m_maxFeatures{ maxFeatures },
                m_keypointBuffer{ maxFeatures, allocator },
                m_descriptorBuffer{ maxFeatures, allocator }
        {}

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        ImageData(CameraIdentity cameraIdentity, size_t maxFeatures, float pyramidScale, size_t numLevels, float imageBorder, const AllocT& allocator, StreamT& stream)
            : ImageData{ cameraIdentity, imageWidth, imageHeight, maxFeatures, pyramidScale, allocator }
        {
            deserialize(stream);
        }

        static constexpr auto members()
        {
            return declare_members(
                &ImageData::m_cameraIdentity,
                &ImageData::m_featureCount,
                &ImageData::m_pyramidScale,
                &ImageData::m_numLevels,
                &ImageData::m_imageBorder,
                &ImageData::m_published,
                &ImageData::m_maxFeatures,
                &ImageData::m_keypointBuffer,
                &ImageData::m_descriptorBuffer
            );
        }

        /*
            Inserts the keypoints into the image data, and returns how many
            items it copied. Because we only support a fixed size, we
            only copy items if we have space.
        */
        size_t Insert(gsl::span<const cv::KeyPoint> keypoints)
        {
            auto toCopy = std::min<int>(gsl::narrow_cast<int>(keypoints.size()), gsl::narrow_cast<int>(m_maxFeatures - m_featureCount));
            m_featureCount += m_keypointBuffer.copy(keypoints.data(), keypoints.data() + toCopy, m_featureCount);
            return toCopy;
        }

        size_t Insert(const cv::KeyPoint& kp)
        {
            if (m_featureCount == m_maxFeatures)
                return 0;

            m_keypointBuffer[m_featureCount] = kp;
            m_featureCount++;
            return 1;
        }

        size_t Insert(const cv::KeyPoint* beg, const cv::KeyPoint* end)
        {
            auto toCopy = std::min<int>(gsl::narrow_cast<int>(end - beg), gsl::narrow_cast<int>(m_maxFeatures - m_featureCount));
            m_featureCount += m_keypointBuffer.copy(beg, beg + toCopy, m_featureCount);
            return toCopy;
        }

        size_t GetMaxFeatures() const
        {
            return m_maxFeatures;
        }

        size_t GetFeatureCount() const
        {
            return m_featureCount;
        }

        float GetPyramidScale() const
        {
            return m_pyramidScale;
        }

        size_t GetNumLevels() const
        {
            return m_numLevels;
        }

        float GetImageBorder() const
        {
            return m_imageBorder;
        }

        CameraIdentity GetCameraIdentity() const
        {
            return m_cameraIdentity;
        }

        const cv::KeyPoint& GetKeypoint(size_t idx) const
        {
            return const_cast<ImageData*>(this)->GetKeypoint(idx);
        }

        cv::KeyPoint& GetKeypoint(size_t idx)
        {
            assert(0 <= idx && idx < m_featureCount);
            return m_keypointBuffer[idx];
        }

        gsl::span<const cv::KeyPoint> GetKeypoints() const
        {
            return { m_keypointBuffer.begin(), m_keypointBuffer.begin() + m_featureCount };
        }

        gsl::span<cv::KeyPoint> GetKeypoints()
        {
            return { m_keypointBuffer.begin(), m_keypointBuffer.begin() + m_featureCount };
        }

        const ORBDescriptor& GetDescriptor(size_t idx) const
        {
            return const_cast<ImageData*>(this)->GetDescriptor(idx);
        }

        ORBDescriptor& GetDescriptor(size_t idx)
        {
            assert(0 <= idx && idx < m_featureCount);
            return m_descriptorBuffer[idx];
        }

        gsl::span<const ORBDescriptor> GetDescriptors() const
        {
            return const_cast<ImageData*>(this)->GetDescriptors();
        }

        gsl::span<ORBDescriptor> GetDescriptors()
        {
            return { m_descriptorBuffer.begin(), m_descriptorBuffer.begin() + m_featureCount };
        }

        void SetDescriptors(gsl::span<const ORBDescriptor> values)
        {
            assert(m_featureCount == values.size());
            m_descriptorBuffer.copy(values.begin(), values.end());
        }

        void ZeroOutDescriptors()
        {
            m_descriptorBuffer.fill(0);
        }

        /*
            Returns whether or not this image was ever published to the map
        */
        bool IsPublished() const
        {
            return m_published;
        }

        void MarkAsPublished() const
        {
            m_published = true;
        }

        template<typename DerivedT>
        static size_t AllocationSizeInBytes(size_t maxFeatures)
        {
            return sizeof(DerivedT) + maxFeatures * sizeof(cv::KeyPoint) + maxFeatures * sizeof(ORBDescriptor);
        }

    private:
        template<typename T>
        using rebind_allocator = typename std::allocator_traits<AllocT>::template rebind_alloc<T>;

        size_t m_featureCount{};
        mutable std::atomic<bool> m_published{ false };

        const size_t m_maxFeatures{};
        const float m_pyramidScale{};
        const size_t m_numLevels{}; //TODO: change numlevels to be reasonable with image resolution
        const float m_imageBorder{};
        const CameraIdentity m_cameraIdentity{};

        buffer<cv::KeyPoint, rebind_allocator<cv::KeyPoint>> m_keypointBuffer{};
        buffer<ORBDescriptor, rebind_allocator<ORBDescriptor>> m_descriptorBuffer{};
    };
}
