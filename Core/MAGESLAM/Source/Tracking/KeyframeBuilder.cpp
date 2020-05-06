// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "KeyframeBuilder.h"
#include "Debugging\SkeletonLogger.h"

using namespace std;

namespace mage
{
    IdGenerator<Keyframe> KeyframeBuilder::s_generator{};

    // The Keyframe proxy is initialized with a set of predicted intrinsics that stays with the image object.

    KeyframeBuilder::KeyframeBuilder(
        const shared_ptr<const AnalyzedImage>& image,
        const mage::Pose& pose,
        const vector<MapPointAssociations<MapPointTrackingProxy>::Association>& mapPoints)
        : KeyframeProxy{ s_generator.generate(), proxy::Image{image}, proxy::Pose{pose}, proxy::Intrinsics{{ ArrayFromMat(image->GetUndistortedCalibration().GetLinearIntrinsics()), image->GetWidth(), image->GetHeight()}}, proxy::Associations<MapPointTrackingProxy>{ image, mapPoints }, proxy::PoseConstraints{ false } }
    {
        SkeletonLogger::ImageLogging::LogImage(*this);
    }

    KeyframeBuilder::KeyframeBuilder(
        const shared_ptr<const AnalyzedImage>& image,
        const mage::Pose& pose) 
        : KeyframeProxy{ s_generator.generate(), proxy::Image{ image }, proxy::Pose{ pose }, proxy::Intrinsics{{ ArrayFromMat(image->GetUndistortedCalibration().GetLinearIntrinsics()), image->GetWidth(),image->GetHeight()}}, proxy::Associations<MapPointTrackingProxy>{ image }, proxy::PoseConstraints{false}}
    {
        SkeletonLogger::ImageLogging::LogImage(*this);
    }
}
