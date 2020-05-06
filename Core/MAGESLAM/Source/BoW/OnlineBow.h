// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseBow.h"
#include "Data\Keyframe.h"

#include <unordered_map>
#include <set>
#include <future>
#include <shared_mutex>

namespace mage
{
    class OnlineBow : public BaseBow
    {
        struct Node;
        struct Entry;

    public:
        OnlineBow(const BagOfWordsSettings& settings, unsigned int numFeatures);
        
        virtual void AddTrainingDescriptors(gsl::span<const ORBDescriptor> descriptors) override;

        virtual void AddImage(const Id<Keyframe>, const AnalyzedImage&) override;

        virtual void RemoveImage(const Id<Keyframe>) override;

        virtual size_t QueryFeatures(const ORBDescriptor& descriptor, const Id<Keyframe>& keyframe, std::vector<ptrdiff_t>& features) const override;
        
        virtual std::unique_ptr<BaseFeatureMatcher> CreateFeatureMatcher(const Id<Keyframe>& id, gsl::span<const ORBDescriptor> features) const override;
        
        virtual std::unique_ptr<BaseBow> CreateTemporaryBow() const override;

        virtual std::vector<QueryMatch> QueryUnknownImage(gsl::span<const ORBDescriptor> descriptors, size_t maxResults) const override;
        
        virtual void Clear() override;

        virtual bool IsTrainingDone() const override;

        ptrdiff_t FindLeafNode(const ORBDescriptor& desc) const;

        ~OnlineBow();

    private:
        OnlineBow(const BagOfWordsSettings& settings, const std::vector<Node>& treeNodes);

        void CompleteTraining();

        void CreateTree(gsl::span<const ORBDescriptor> descriptors);

        void SetNodeWeights(gsl::span<const ORBDescriptor> training_features);

        void InitializeTraining(gsl::span<const ORBDescriptor> descriptors, std::vector<ORBDescriptor>& clusters);

        void InsertDescriptors(const Id<Keyframe>& id, gsl::span<const ORBDescriptor> descriptors);

        void Kmean(unsigned int parent_id, gsl::span<const ORBDescriptor> descriptors, unsigned int current_level);

        void Kmedoid(unsigned int parent_id, gsl::span<const ORBDescriptor> descriptors, unsigned int current_level);

        void KmeanCenter(gsl::span<const ORBDescriptor> descriptors, gsl::span<const unsigned int> indexes, ORBDescriptor& kmean);

        void IterateClusteringKmean(gsl::span<const ORBDescriptor> descriptors, std::vector<ORBDescriptor>& kmeans, std::vector<std::vector<unsigned int>>& groups);

        void IterateClusteringKmedoid(gsl::span<const ORBDescriptor> descriptors, std::vector<ORBDescriptor>& kmedoids, std::vector<std::vector<unsigned int>>& groups);

        int FindCluster(const ORBDescriptor& desc, const std::vector<ORBDescriptor>& descriptors);

        std::vector<Node> m_nodes;
        std::vector<ORBDescriptor> m_training;
        std::vector<ptrdiff_t> m_descriptorsCount;
        std::set<Id<Keyframe>> m_imageSet;

        //A map of NodeId <==> (a map of frameID <==> Entry)
        std::unordered_map<unsigned int, std::unordered_map<Id<Keyframe>, Entry>> m_NodeKeyframeMap;

        bool m_isTrainingDone{ false };
        mutable std::shared_mutex m_mutex;
    };
}
