// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <atomic>
#include <arcana/threading/cancellation.h>

#include "BoW/BaseBow.h"
#include "Debugging/Introspection.h"
#include "Image/ImageFactory.h"
#include "Map/ThreadSafeMap.h"
#include "Map/ThreadSafePoseHistory.h"
#include "Device/IMUCharacterization.h"
#include "Schedule.h"


namespace mage
{
    using ImageFactoryMap = std::map<CameraIdentity, std::unique_ptr<ImageFactory>>;

    struct MageContext
    {
        /*
            The global image factory that allocates our images.
        */
        ImageFactoryMap& ImageFactoryMap;

        /*
            The BagOfWords used for the program
        */
        BaseBow& BagOfWords;

        /*
            The map instance that is shared throughout the algorithm
        */
        ThreadSafeMap& Map;

        /*
            The pose history that keeps track of all the poses output from
            mage and updates them based on how the map gets adjusted.
        */
        ThreadSafePoseHistory& History;

        /*
            The state machine scheduler
        */
        mira::state_machine_observer StateMachine;

        /*
            Our introspection system to peek under the hood from our tooling.
        */
        Introspection Introspection{};

        /*
            Stores device specific settings for imu
        */
        const device::IMUCharacterization& IMUCharacterization;

        MageContext(
            mage::ImageFactoryMap& imageFactory,
            BaseBow& bagOfWords,
            ThreadSafeMap& threadSafeMap,
            ThreadSafePoseHistory& threadSafeHistory,
            const device::IMUCharacterization& imuCharacterization,
            mira::state_machine_driver& driver)
            :   ImageFactoryMap{ imageFactory },
                BagOfWords{ bagOfWords },
                Map{ threadSafeMap },
                History{ threadSafeHistory },
                StateMachine{ driver },
                IMUCharacterization{ imuCharacterization }
        {}

        MageContext(const MageContext&) = delete;

        enum class ThreadType
        {
            TrackingThread,
            MappingThread,
            LoopClosureThread
        };

        void TickImageFactories(ThreadType thread)
        {
            for (auto& imageFactory : ImageFactoryMap)
            {
                switch (thread)
                {
                case ThreadType::MappingThread:
                    imageFactory.second->TickMappingThread();
                    break;
                case ThreadType::TrackingThread:
                    imageFactory.second->TickTrackingThread();
                    break;
                case ThreadType::LoopClosureThread:
                    imageFactory.second->TickLoopClosureThread();
                    break;
                default:
                    assert(false && "Unexpected ThreadType");
                }
            }
        }
    };
}
