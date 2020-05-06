// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Device/CameraCalibration.h"

#include "MAGESlam.h"
#include "Pose.h"
#include "Data.h"

#include "Fuser\Fuser.h"
#include "Image\ImageFactory.h"

#include <memory>
#include <future>
#include <chrono>

namespace mage
{
    struct FrameData
    {
        MAGESlam::FrameFormat Format;
        ImageHandle ImageData;
        Fuser& Fuser;
        bool AddedToFilter;
        cv::Mat ImageMat;

        FrameData(
            const MAGESlam::FrameFormat& format,
            ImageHandle imageData,
            cv::Mat imageMat,
            mage::Fuser& fuser,
            bool addedToFilter)
            : Format{ format },
            ImageData{ move(imageData) },
            ImageMat { imageMat },
            Fuser{ fuser },
            AddedToFilter {addedToFilter}
        {}

        void Set(MAGESlam::Tracking&& tracking)
        {
            assert(!m_set && "tried setting the result multiple times");

            m_set = true;
            m_tracking.set_value(std::move(tracking));
        }

        std::future<MAGESlam::Tracking> GetFuture()
        {
            return m_tracking.get_future();
        }

        ~FrameData()
        {
            if (AddedToFilter)
            {
                Fuser.RemoveImageFence(Format.Timestamp);
            }

            assert(m_set && "lost an input frame without setting it.");
        }

    private:
        std::promise<MAGESlam::Tracking> m_tracking{};
        bool m_set{ false };
    };

}
