// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Types.h"
#include "MageSettings.h"
#include "Image\ORBDescriptor.h"
#include "BaseFeatureMatcher.h"

#include <vector>
#include <gsl\gsl>
#include <memory>
  
namespace mage
{
    class Keyframe;
    class AnalyzedImage;

    class BaseBow
    {
    public:
        struct QueryMatch
        {
            Id<Keyframe> Id;
            float Score;

            operator mage::Id<Keyframe>() const
            {
                return Id;
            }
        };

        BaseBow(const BagOfWordsSettings& settings)
            :m_settings{ settings }
        {}

        virtual ~BaseBow() {}

        virtual void AddTrainingDescriptors(gsl::span<const ORBDescriptor>) {}

        virtual void AddImage(const Id<Keyframe>, const AnalyzedImage& image) = 0;

        virtual void RemoveImage(const Id<Keyframe>) = 0;

        virtual size_t QueryFeatures(const ORBDescriptor& descriptor, const Id<Keyframe>& keyframe, std::vector<ptrdiff_t>& features) const = 0;

        virtual std::unique_ptr<BaseFeatureMatcher> CreateFeatureMatcher(const Id<Keyframe>& id, gsl::span<const ORBDescriptor> features) const = 0;
        
        virtual std::unique_ptr<BaseBow> CreateTemporaryBow() const = 0;

        virtual std::vector<QueryMatch> QueryUnknownImage(gsl::span<const ORBDescriptor> descriptors, size_t maxResults) const = 0;

        virtual void Clear() = 0;

        virtual bool IsTrainingDone() const { return true;}

    protected:
        const BagOfWordsSettings& m_settings;
    };
}
