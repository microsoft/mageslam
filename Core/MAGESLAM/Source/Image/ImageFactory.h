// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "ImageData.h"

#include <mutex>
#include <list>
#include <atomic>

namespace mage
{
    using ImageHandle = std::unique_ptr<ImageData<ImageAllocator>, std::function<void(const ImageData<ImageAllocator>*)>>;
    using ConstImageHandle = std::unique_ptr<const ImageData<ImageAllocator>, std::function<void(const ImageData<ImageAllocator>*)>>;

    class ImageFactory
    {
        // mage::std_allocator only keeps a pointer to the allocation strategy,
        // in order to keep it around we have to manage the lifetime of it.
        // We also need to make sure we destroy the allocation strategy only after the ~ImageData runs,
        // this is why we privately inherit from unique_ptr so that it gets destroyed in the right order
        struct InternalData : private std::unique_ptr<block_splitting_allocation_strategy>, public ImageData<ImageAllocator>
        {
        public:
            InternalData(CameraIdentity cameraIdentity, size_t maxFeatures, float pyramidScale, size_t numLevels, float borderSize,
                const ImageAllocator& allocator, std::unique_ptr<block_splitting_allocation_strategy> strategy)
                :   std::unique_ptr<block_splitting_allocation_strategy>(std::move(strategy)),
                    ImageData(cameraIdentity, maxFeatures, pyramidScale, numLevels, borderSize, allocator)
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            InternalData(CameraIdentity cameraIdentity, size_t maxFeatures, float pyramidScale, size_t numLevels, float borderSize,
                const ImageAllocator& allocator, std::unique_ptr<block_splitting_allocation_strategy> strategy, StreamT& stream)
                :   std::unique_ptr<block_splitting_allocation_strategy>(std::move(strategy)),
                    ImageData(cameraIdentity, maxFeatures, pyramidScale, numLevels, borderSize, allocator, stream)
            {}

            size_t TrackingIterationOnDeallocation{ 0 };
            size_t MappingIterationOnDeallocation{ 0 };
            size_t LoopClosureIterationOnDeallocation{ 0 };
        };
    public:
        ImageFactory(CameraIdentity cameraIdentity, size_t maxFeatures, float pyramidScale, size_t numLevels, float borderSize, size_t numPreAllocatedImages)
            :   m_allocationSize{ InternalData::AllocationSizeInBytes<InternalData>(maxFeatures) },
                m_strategy{
                    m_allocationSize, // each element in the free_list should be of our element size
                    numPreAllocatedImages * m_allocationSize
                },
                m_cameraIdentity{ cameraIdentity },
                m_maxFeatures{ maxFeatures },
                m_pyramidScale{ pyramidScale },
                m_numLevels{ numLevels },
                m_borderSize{ borderSize }
        {}
        
        ~ImageFactory()
        {
            for (auto& el : m_availableForDeallocation)
            {
                memory::factory::destroy(el);
                m_strategy.deallocate({ el, m_allocationSize });
            }
            m_availableForDeallocation.clear();
        }

        ImageHandle AllocateMetaData()
        {
            memory::block memory;
            {
                std::lock_guard<std::mutex> guard{ m_mutex };
                memory = m_strategy.allocate(m_allocationSize);
            }
            auto strategy = std::make_unique<block_splitting_allocation_strategy>(memory);
            ImageAllocator::rebind<InternalData> alloc{ *strategy };
            ImageHandle handle = ImageHandle{ alloc.allocate(1), m_deallocator };
            memory::factory::construct(static_cast<InternalData*>(handle.get()), m_cameraIdentity, m_maxFeatures, m_pyramidScale, m_numLevels, m_borderSize, alloc, move(strategy));
            return handle;
        }

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        ImageHandle AllocateMetaData(StreamT& stream)
        {
            memory::block memory;
            {
                std::lock_guard<std::mutex> guard{ m_mutex };
                memory = m_strategy.allocate(m_allocationSize);
            }
            auto strategy = std::make_unique<block_splitting_allocation_strategy>(memory);
            ImageAllocator::rebind<InternalData> alloc{ *strategy };
            ImageHandle handle = ImageHandle{ alloc.allocate(1), m_deallocator };
            memory::factory::construct(static_cast<InternalData*>(handle.get()), m_maxFeatures, alloc, move(strategy), stream);
            return handle;
        }

        void TickTrackingThread()
        {
            ++m_trackingIteration;
        }

        void TickMappingThread()
        {
            ++m_mappingIteration;
        }

        void TickLoopClosureThread()
        {
            ++m_loopClosureIteration;
        }

        void CleanupImages()
        {
            // TODO PERF, we could enforce Cleanup and Allocate to only
            // be called from one thread (which is what we're doing anyway)
            // and split the m_availableForDeallocation manipulation from the
            // destroy and deallocate by copying our pointers to a temporary vector
            // to reduce the lock time
            
            std::lock_guard<std::mutex> guard{ m_mutex };

            auto newEnd = std::partition(m_availableForDeallocation.begin(), m_availableForDeallocation.end(), [&](InternalData* data)
            {
                bool canDelete = false;
                // if an image was never published we don't have to
                // wait for the iterations to loop over as no one
                // has any data to this object
                if (!data->IsPublished())
                {
                    canDelete = true;
                }
                else
                {
                    // if it has been published we have to wait till each thread
                    // has completed at least an iteration since the iteration at which
                    // it was destroyed to make sure no one has references to map points
                    // referring to the image.
                    canDelete = m_trackingIteration > data->TrackingIterationOnDeallocation &&
                        m_mappingIteration > data->MappingIterationOnDeallocation &&
                        m_loopClosureIteration > data->LoopClosureIterationOnDeallocation;
                }

                // invert the predicate because partition orders elements
                // where predicate = true, before predicate = false
                return !canDelete;
            });

            for (auto itr = newEnd; itr != m_availableForDeallocation.end(); ++itr)
            {
                memory::factory::destroy(*itr);
                m_strategy.deallocate({ *itr, m_allocationSize });
            }

            m_availableForDeallocation.erase(newEnd, m_availableForDeallocation.end());
        }

    private:
        std::mutex m_mutex;
        const size_t m_allocationSize;

        image_allocation_strategy m_strategy;

        std::atomic<size_t> m_trackingIteration{ 0 };
        std::atomic<size_t> m_mappingIteration{ 0 };
        std::atomic<size_t> m_loopClosureIteration{ 0 };

        std::vector<InternalData*> m_availableForDeallocation;

        const CameraIdentity m_cameraIdentity;
        const size_t m_maxFeatures;

        const float m_pyramidScale;
        const size_t m_numLevels;
        const float m_borderSize;

        const std::function<void(const ImageData<ImageAllocator>*)> m_deallocator = [this](const ImageData<ImageAllocator>* image)
        {
            InternalData* allocated = const_cast<InternalData*>(static_cast<const InternalData*>(image));
            allocated->TrackingIterationOnDeallocation = m_trackingIteration;
            allocated->MappingIterationOnDeallocation = m_mappingIteration;
            allocated->LoopClosureIterationOnDeallocation = m_loopClosureIteration;

            {
                std::lock_guard<std::mutex> guard{ m_mutex };

                // if it hasn't been published we can fast path and just destroy it immediately
                if (!allocated->IsPublished())
                {
                    memory::factory::destroy(allocated);
                    m_strategy.deallocate({ allocated, m_allocationSize });
                }
                else
                {
                    m_availableForDeallocation.push_back(allocated);
                    assert(std::unique(m_availableForDeallocation.begin(), m_availableForDeallocation.end()) == m_availableForDeallocation.end());
                }
            }
        };
    };
}
