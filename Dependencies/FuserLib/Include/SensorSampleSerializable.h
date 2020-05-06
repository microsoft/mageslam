// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <cereal\cereal.hpp>
#include <cereal\types\chrono.hpp>
#include <cereal\types\vector.hpp>
#include <cereal\archives\portable_binary.hpp>
#include "SensorSample.h"

namespace mage
{
    struct IMUFile
    {
        std::vector<SensorSample> Samples;

        template <class Archive>
        void serialize(Archive & ar, std::uint32_t const /*version*/)
        {
            ar(CEREAL_NVP(Samples));
        }
    };
}
CEREAL_CLASS_VERSION(mage::IMUFile, 1);
