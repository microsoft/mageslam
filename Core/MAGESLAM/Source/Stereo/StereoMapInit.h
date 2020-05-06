// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "utils\thread_memory.h"
#include "arcana\analysis\determinator.h"

#include <opencv2\core\types.hpp>
#include <boost\optional.hpp>

namespace UnitTests
{
    class StereoUnitTest;
}

namespace mage
{
    class AnalyzedImage;
    struct StereoMapInitializationSettings;
    struct InitializationData;

    class StereoMapInit
    {
    public:
        StereoMapInit(const StereoMapInitializationSettings& settings, mira::determinator& determinator);
        boost::optional<InitializationData> Initialize(const std::shared_ptr<AnalyzedImage>& frame0, const std::shared_ptr<AnalyzedImage>& frame1, const cv::Matx44f& frame0ToFrame1, thread_memory memory);

    private:

        const StereoMapInitializationSettings& m_settings;
        mira::determinator& m_determinator;

        friend ::UnitTests::StereoUnitTest;
    };
}
