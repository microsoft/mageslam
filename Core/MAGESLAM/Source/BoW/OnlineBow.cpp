// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "OnlineBow.h"
#include "OnlineBowFeatureMatcher.h"

#include "Tracking\FeatureMatcher.h"

#include <arcana\iterators.h>
#include <arcana\threading\cancellation.h>

#include <random>
#include <memory>

using namespace std;

namespace mage
{
    struct OnlineBow::Node
    {
        unsigned int nodeID;
        vector<unsigned int> childrenIDs;
        unsigned int parentID;

        // The weight of a node reflects the frequency of occurence of a Node in the training set
        // Currently use Inverse document frequency (IDF) to calculate the weight
        // log ((Number of total training images) / (Number of images which have descriptors belong to the Node)) 
        float weight;

        // Representative feature descriptor of the Node
        ORBDescriptor descriptor;

        Node(unsigned int nID)
            :nodeID{ nID }
        {}

        Node(unsigned int nID, unsigned int pID, const ORBDescriptor& desc, float value = 0.0)
            : nodeID{ nID },
            parentID{ pID },
            descriptor{ desc },
            weight{ value }
        {}
    };

    //Every (Node <--> keyframe) pair has an Entry
    struct OnlineBow::Entry
    {
        // Each keyframe associated with a Node has a "nodeValue",
        // based on how many descriptors of the keyframe is assigned to the Node, and the Node's weight
        float nodeValue;

        // The feature descriptors indexes of the keyframe assigned to the Node
        vector<ptrdiff_t> indexes;

        Entry(float value)
        {
            nodeValue = value;
        }
    };

    OnlineBow::OnlineBow(const BagOfWordsSettings& settings, unsigned int numFeatures)
        : BaseBow{ settings }
    {
        m_training.reserve(m_settings.TrainingFrames * numFeatures);
    }

    OnlineBow::OnlineBow(const BagOfWordsSettings& settings, const vector<Node>& treeNodes)
        : BaseBow{ settings }
    {
        for (const auto& node : treeNodes)
        {
            m_nodes.push_back(node);
            m_NodeKeyframeMap.insert({ node.nodeID, {} });
        }
    }

    void OnlineBow::AddTrainingDescriptors(gsl::span<const ORBDescriptor> descriptors)
    {
        unique_lock<shared_mutex> lock{ m_mutex };
        if (!m_isTrainingDone)
        {
            copy(descriptors.begin(), descriptors.end(), mira::emplace_inserter(m_training));

            // There are cases when the number of descriptors is not equal to "NumFeatures" of the FeatureExtractorSettings.
            m_descriptorsCount.push_back(descriptors.size());

            if (m_descriptorsCount.size() >= m_settings.TrainingFrames && m_training.size() >= m_settings.MinTrainingSize)
            {
                 CompleteTraining();
            }
        }
    }

    void OnlineBow::AddImage(const Id<Keyframe> id, const AnalyzedImage& image)
    {
        unique_lock<shared_mutex> lock{ m_mutex };
        InsertDescriptors(id, image.GetDescriptors());
    }

    void OnlineBow::RemoveImage(const Id<Keyframe> id)
    {
        unique_lock<shared_mutex> lock{ m_mutex };

        for (auto& pair : m_NodeKeyframeMap)
        {
            auto entry = pair.second.find(id);
            if (entry != pair.second.end())
            {
                pair.second.erase(entry);
            }
        }
        m_imageSet.erase(id);
    }

    size_t OnlineBow::QueryFeatures(const ORBDescriptor& desc, const Id<Keyframe>& kfId, vector<ptrdiff_t>& features) const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        assert(m_imageSet.find(kfId) != m_imageSet.end() && "we should never query for a keyframe that isn't in the BOW");

        features.clear();

        unsigned int nodeID = FindLeafNode(desc);
        auto kf = m_NodeKeyframeMap.find(nodeID)->second.find(kfId);

        if (kf == m_NodeKeyframeMap.find(nodeID)->second.end())
        {
            return 0;
        }

        copy(kf->second.indexes.begin(), kf->second.indexes.end(), mira::emplace_inserter(features));
        
        return features.size();
    }

    unique_ptr<BaseFeatureMatcher> OnlineBow::CreateFeatureMatcher(const Id<Keyframe>& id, gsl::span<const ORBDescriptor> features) const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        return make_unique<OnlineBowFeatureMatcher>(*this, id, features);
    }

    std::unique_ptr<BaseBow> OnlineBow::CreateTemporaryBow() const
    {
       shared_lock<shared_mutex> lock{ m_mutex };
       unique_ptr<OnlineBow> tempBow(new OnlineBow(m_settings, m_nodes));
       return tempBow;
    }

    /*
    QueryUnknownImage is used to find previously added keyframes which are similar to the current unknown image.
    First it uses the descriptors of the unknown image to create a map of leafNodeID <==> leafNodeValue to represent the image.
    Since every added keyframe has a nodeValue for every leafNode which it has descriptors assigned to (see OnlineBow::Entry struct),
    we use the similarity between a keyframe's nodeValues and the unknown image's nodeValues of the leafNodes they both associated with to 
    calculate a similarity score for all possible keyframes.
    */
    vector<BaseBow::QueryMatch> OnlineBow::QueryUnknownImage(gsl::span<const ORBDescriptor> descriptors, size_t maxResults) const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        vector<QueryMatch> results;

        // Create a map of <nodeId, nodeValue> for the current query image
        unordered_map<unsigned int, float> curMap;
        float sum = 0;

        for (ptrdiff_t i = 0; i < descriptors.size(); ++i)
        {
            const auto& desc = descriptors[i];
            const auto& leafNode = m_nodes[FindLeafNode(desc)];
            auto it = curMap.find(leafNode.nodeID);

            if (it == curMap.end())
            {
                curMap.emplace(leafNode.nodeID, leafNode.weight);
            }
            else
            {
                it->second += leafNode.weight;
            }

            sum += leafNode.weight;
        }

        if (sum == 0)
            return results;

        // Normalize
        for (auto& entry : curMap)
        {
            entry.second = entry.second / sum;
        }

        // A map of KeyFrameID <==> score for the existing KeyFrames
        unordered_map<Id<Keyframe>, float> map;

        // Iterate all the Nodes in the created map of the current unknown image
        // Calculate a score for all the keyframes which are associated with those Nodes
        // Using L1_NORM to calculate similarity for now
        for (auto it = curMap.begin(); it != curMap.end(); ++it)
        {
            unsigned int nodeId = it->first;
            float imageNodeValue = it->second;

            for (const auto& entry : m_NodeKeyframeMap.find(nodeId)->second)
            {
                Id<Keyframe> keyFrameId = entry.first;
                float keyFrameNodeValue = entry.second.nodeValue;

                //L1_NORM
                float value = fabs(imageNodeValue - keyFrameNodeValue) - fabs(imageNodeValue) - fabs(keyFrameNodeValue);

                //L2_NORM
                //float value = -nodeValue * keyFrameValue;

                auto mapIt = map.find(keyFrameId);
                if (mapIt == map.end())
                {
                    map.emplace(keyFrameId, value);
                }
                else
                {
                    mapIt->second += value;
                }
            }
        }

        //transfer the score to 0 - 1, the bigger value the similar
        float maxScore = 0;
        for (auto& pair : map)
        {
            //L1_norm
            pair.second = -pair.second / 2.0F;

            //L2_norm
            /*if (sit->second <= -1.0) // rounding error
            sit->second = 1.0;
            else
            sit->second = 1.0 - sqrt(1.0 + sit->second); // [0..1]*/

            if (pair.second > maxScore)
            {
                maxScore = pair.second;
            }
        }

        // Put the map in a vector and sort it according to the score
        vector<pair<Id<Keyframe>, float>> sortedvector;
        sortedvector.reserve(map.size());
        float qualifyingScore = maxScore * m_settings.QualifyingCandidateScore;

        copy_if(map.begin(), map.end(), back_inserter(sortedvector), [qualifyingScore](const pair<Id<Keyframe>, float>& entry)
        {
            return entry.second >= qualifyingScore;
        });

        // The one with higher score is in front of the one with lower score
        // The one with higher score means it is more similar to the current image
        sort(sortedvector.begin(), sortedvector.end(), [](const pair<Id<Keyframe>, float>& a, const pair<Id<Keyframe>, float>& b)
        {
            return a.second > b.second;
        });

        auto end = sortedvector.end();
        if (distance(sortedvector.begin(), end) > gsl::narrow_cast<ptrdiff_t>(maxResults))
        {
            end = sortedvector.begin() + maxResults;
        }

        results.reserve(distance(sortedvector.begin(), end));
        transform(sortedvector.begin(), end, back_inserter(results), [](const pair<Id<Keyframe>, float>& pair) { return QueryMatch{ pair.first, pair.second }; });

        return results;
    }

    void OnlineBow::Clear()
    {
        unique_lock<shared_mutex> lock{ m_mutex };
        m_imageSet.clear();
        m_descriptorsCount.clear();
        m_training.clear();
        m_nodes.clear();
        m_isTrainingDone = false;
    }

    bool OnlineBow::IsTrainingDone() const
    {
        shared_lock<shared_mutex> lock{ m_mutex };
        return m_isTrainingDone;
    }

    ptrdiff_t OnlineBow::FindLeafNode(const ORBDescriptor& desc) const
    {
        ptrdiff_t curId = 0;

        // Go down tree until the current node is the leaf node.
        while (!m_nodes[curId].childrenIDs.empty())
        {
            auto best_d = numeric_limits<int>::max();

            // Find the child node, whose descriptor is closest to the desc.
            for (unsigned int childIdx : m_nodes[curId].childrenIDs)
            {
                auto d = GetDescriptorDistance(desc, m_nodes[childIdx].descriptor);
                if (d < best_d)
                {
                    best_d = d;
                    curId = childIdx;
                }
            }
        }

        return curId;
    }

    void OnlineBow::CompleteTraining()
    {
        if (m_training.size() != 0)
        {
            CreateTree(m_training);
        }

        m_training.clear();

        m_isTrainingDone = true;
    }

    void OnlineBow::CreateTree(gsl::span<const ORBDescriptor> descriptors)
    {
        m_nodes.clear();

        // "TrainingTreeLevels" represents how many levels below the root node, so the tree has total "TrainingTreeLevels + 1" levels.
        unsigned int expected_nodes = (unsigned int)((pow((float)m_settings.TrainingTreeBranchingFactor, (float)m_settings.TrainingTreeLevels + 1) - 1) / (m_settings.TrainingTreeBranchingFactor - 1));
        m_nodes.reserve(expected_nodes);

        m_nodes.push_back(Node(0));

        Kmean(0, descriptors, 1);

        SetNodeWeights(m_training);
    }

    void OnlineBow::SetNodeWeights(gsl::span<const ORBDescriptor> training_features)
    {
        const size_t nImages = m_descriptorsCount.size();

        // A map of leafNodeID <==> the number of training image assigned to this LeafNode
        unordered_map<unsigned int, unsigned int> leafNodeImageMap;
        
        // Every training image will assign its descriptors to a set of leaf nodes
        // leafNodeFound is the set of leafNode IDs, which are associated with every training image.
        set<unsigned int> leafNodeFound;

        // "start" represents every training image's first descriptor's index in "m_training" 
        unsigned int start = 0;

        for (unsigned int imageIndex = 0; imageIndex < nImages; start += m_descriptorsCount[imageIndex], imageIndex++)
        {
            // Clear the set for every training image
            leafNodeFound.clear();

            // Assign every descriptor of the current training image to a leafNode
            for (int j = 0; j < m_descriptorsCount[imageIndex]; j++)
            {
                unsigned int leafNodeID = FindLeafNode(training_features[start + j]);

                /* 
                If the leafNodeID is not in the leafNodeFound yet, which means this is the first descriptor of the training image assigned to 
                the leafNode, then leafNodeImageMap's corresponding entry, whose key is leafNodeID, increases 1 to its mapped value to indicate
                that there is one more training image has descriptors associated with the leafNode.
                If the leafNodeID is already in the leafNodeFound set, which means the current training image has already be added to the leafNodeImageMap,
                then we do not need to do anything.
                */
                if (leafNodeFound.find(leafNodeID) == leafNodeFound.end())
                {
                    auto it = leafNodeImageMap.find(leafNodeID);
                    if (it == leafNodeImageMap.end())
                    {
                        leafNodeImageMap.insert(pair<unsigned int, unsigned int>(leafNodeID, 1));
                    }
                    else
                    {
                        it->second++;
                    }
                    leafNodeFound.insert(leafNodeID);
                }
            }
        }

        //Calculate the IDF (Inverse document frequency) of every leafNode
        //log((Number of total training images) / (Number of images which have descriptors belong to the Node))
        for (auto it = leafNodeImageMap.begin(); it != leafNodeImageMap.end(); ++it)
        {
            // Use (nImages + 1) here to avoid Node weight equals to 0 
            m_nodes[it->first].weight = log((float)(nImages + 1) / (float)it->second);
        }
    }

    void OnlineBow::InitializeTraining(gsl::span<const ORBDescriptor> descriptors, vector<ORBDescriptor>& clusters)
    {
        vector<ORBDescriptor::Ref> refs;
        transform(descriptors.begin(), descriptors.end(), mira::emplace_inserter(refs), [](const auto& desc)
        {
            return desc.AsRef();
        });

        shuffle(refs.begin(), refs.end(), mt19937{ /*random_device{}()*/ });


        for (size_t i = 0; i < m_settings.TrainingTreeBranchingFactor && i < refs.size(); i++)
        {
            clusters.push_back(*refs[i]);
        }
    }

    void OnlineBow::InsertDescriptors(const Id<Keyframe>& id, gsl::span<const ORBDescriptor> descriptors)
    {
        // store the address of all the nodeValue, normalize the nodeValues in the end 
        vector<float*> pts;
        
        float sum = 0;

        for (ptrdiff_t i = 0; i < descriptors.size(); ++i)
        {
            const auto& desc = descriptors[i];
            unsigned int nodeId = FindLeafNode(desc);
            auto pair = m_NodeKeyframeMap.find(nodeId);
            auto entry = pair->second.find(id);

            if (entry == pair->second.end())
            {
                pair->second.insert({ id,{ 0.f } });
                entry = pair->second.find(id);
                pts.emplace_back(&(entry->second.nodeValue));
            }

            entry->second.indexes.push_back(i);
            entry->second.nodeValue += m_nodes[nodeId].weight;
            sum += m_nodes[nodeId].weight;
        }

        if (sum == 0)
            return;

        // Normalize
        for (auto it = pts.begin(); it != pts.end(); ++it)
        {
            **it = **it / sum;
        }

        m_imageSet.emplace(id);
    }

    void OnlineBow::Kmean(unsigned int parent_id, gsl::span<const ORBDescriptor> descriptors, unsigned int current_level)
    {
        vector<ORBDescriptor> kmeans;
        kmeans.reserve(m_settings.TrainingTreeBranchingFactor);
        InitializeTraining(descriptors, kmeans);
        vector<vector<unsigned int>> groups;
        IterateClusteringKmean(descriptors, kmeans, groups);
        
        for (auto& mean : kmeans)
        {
            unsigned int id = gsl::narrow_cast<unsigned int>(m_nodes.size());
            m_nodes.emplace_back(id, parent_id, mean);
            m_nodes[parent_id].childrenIDs.push_back(id);
            m_NodeKeyframeMap.insert({ id, {} });
        }

        if (current_level < m_settings.TrainingTreeLevels)
        {
            // Iterate again with the resulting clusters
            for (unsigned int i = 0; i < kmeans.size(); ++i)
            {
                unsigned int id = m_nodes[parent_id].childrenIDs[i];

                vector<ORBDescriptor> child_features;
                child_features.reserve(groups[i].size());

                transform(groups[i].begin(), groups[i].end(), back_inserter(child_features), [descriptors](unsigned int idx) { return descriptors[idx]; });

                if (child_features.size() > 1)
                {
                    Kmean(id, child_features, current_level + 1);
                }
            }
        }
    }

    void OnlineBow::Kmedoid(unsigned int parent_id, gsl::span<const ORBDescriptor> descriptors, unsigned int current_level)
    {
        vector<ORBDescriptor> kmedoids;
        kmedoids.reserve(m_settings.TrainingTreeBranchingFactor);
        InitializeTraining(descriptors, kmedoids);
        vector<vector<unsigned int>> groups;
        IterateClusteringKmedoid(descriptors, kmedoids, groups);

        for (unsigned int i = 0; i < kmedoids.size(); ++i)
        {
            unsigned int id = gsl::narrow_cast<unsigned int>(m_nodes.size());
            m_nodes.emplace_back(id, parent_id, kmedoids[i]);
            m_nodes[parent_id].childrenIDs.push_back(id);
            m_NodeKeyframeMap.insert({ id, {} });
        }

        if (current_level < m_settings.TrainingTreeLevels)
        {
            // Iterate again with the resulting clusters
            for (unsigned int i = 0; i < kmedoids.size(); ++i)
            {
                unsigned int id = m_nodes[parent_id].childrenIDs[i];

                vector<ORBDescriptor> child_features;
                child_features.reserve(groups[i].size());

                transform(groups[i].begin(), groups[i].end(), back_inserter(child_features), [descriptors](unsigned int idx) { return descriptors[idx]; });

                if (child_features.size() > 1)
                {
                    Kmedoid(id, child_features, current_level + 1);
                }
            }
        }
    }

    void OnlineBow::KmeanCenter(gsl::span<const ORBDescriptor> descriptors, gsl::span<const unsigned int> indexes, ORBDescriptor& kmean)
    {
        assert(!descriptors.empty() && "No descriptor assigned to the current KmeanCenter");

        vector<unsigned int> sum(ORBDescriptor::COLUMNS * ORBDescriptor::BITS_PER_COLUMN, 0);

        for (ptrdiff_t i = 0; i < indexes.size(); i++)
        {
            const uint8_t* pt = descriptors[indexes[i]].Data();
            for (int j = 0; j < ORBDescriptor::COLUMNS; ++j, ++pt)
            {
                for (int k = 0; k < ORBDescriptor::BITS_PER_COLUMN; k++)
                {
                    sum[j * ORBDescriptor::BITS_PER_COLUMN + k] += (*pt >> k) & 0b1;
                }
            }
        }

        uint8_t* mpt = kmean.Data();
        unsigned int halfSize = (indexes.size() + 1) / 2;

        for (size_t i = 0; i < ORBDescriptor::COLUMNS; ++i, ++mpt)
        {
            for (int j = 0; j < ORBDescriptor::BITS_PER_COLUMN; j++)
            {
                if (sum[i * ORBDescriptor::BITS_PER_COLUMN + j] >= halfSize)
                {
                    *mpt |= 1 << j;
                }
                else
                {
                    *mpt &= ~(1 << j);
                }
            }
        }
    }

    void OnlineBow::IterateClusteringKmean(gsl::span<const ORBDescriptor> descriptors, vector<ORBDescriptor>& kmeans, vector<vector<unsigned int>>& groups)
    {
        unsigned int changedKmeanCounter = 0;
        unsigned int iterationCounter = 0;

        do // Repeat until kmean centers stay the same or it hits m_settings.MaxTrainingIteration
        {
            groups.clear();
            groups.resize(kmeans.size(), vector<unsigned int>());
            changedKmeanCounter = 0;
            iterationCounter++;

            for (ptrdiff_t i = 0; i < descriptors.size(); ++i)
            {
                int index = FindCluster(descriptors[i], kmeans);
                groups[index].push_back(i);
            }

            for (size_t g_index = 0; g_index < groups.size(); g_index++)
            {
                ORBDescriptor preDesc = kmeans[g_index];
                KmeanCenter(descriptors, groups[g_index], kmeans[g_index]);
                if (GetDescriptorDistance(preDesc, kmeans[g_index]) != 0)
                {
                    changedKmeanCounter++;
                }
            }
        } while (iterationCounter < m_settings.MaxTrainingIteration && changedKmeanCounter > 0);
    }

    void OnlineBow::IterateClusteringKmedoid(gsl::span<const ORBDescriptor> descriptors, vector<ORBDescriptor>& kmedoids, vector<vector<unsigned int>>& groups)
    {
        unsigned int changedKmediodCounter = 0;
        unsigned int iterationCounter = 0;

        do  // Repeat until the kmediods stay the same or it hits m_settings.MaxTrainingIteration
        {
            groups.clear();
            groups.resize(kmedoids.size(), vector<unsigned int>());
            changedKmediodCounter = 0;
            iterationCounter++;

            for (ptrdiff_t i = 0; i < descriptors.size(); ++i)
            {
                int index = FindCluster(descriptors[i], kmedoids);
                groups[index].push_back(i);
            }

            for (size_t g_index = 0; g_index < groups.size(); g_index++)
            {

                int64_t minD = numeric_limits<int64_t>::max();
                size_t minI = 0;

                // TODO PERF make this better
                for (size_t i = 0; i < groups[g_index].size(); ++i)
                {
                    auto& desc = descriptors[groups[g_index][i]];

                    int64_t distance = 0;
                    for (size_t k = 0; k < groups[g_index].size(); ++k)
                    {
                        distance += GetDescriptorDistance(desc, descriptors[groups[g_index][k]]);
                    }

                    if (distance < minD)
                    {
                        minD = distance;
                        minI = i;
                    }
                }

                if (GetDescriptorDistance(descriptors[groups[g_index][minI]], kmedoids[g_index]) != 0)
                {
                    changedKmediodCounter++;
                    (descriptors[groups[g_index][minI]]).CopyTo(kmedoids[g_index]);
                }
            }
        } while (iterationCounter < m_settings.MaxTrainingIteration && changedKmediodCounter > 0);
    }

    int OnlineBow::FindCluster(const ORBDescriptor& desc, const vector<ORBDescriptor>& centers)
    {
        auto minDistance = min_element(centers.begin(), centers.end(), [&desc](const ORBDescriptor& center1, const ORBDescriptor& center2)
        {
            return GetDescriptorDistance(center1, desc) < GetDescriptorDistance(center2, desc);
        });
        return distance(centers.begin(), minDistance);
    }

    OnlineBow::~OnlineBow()
    {}
}
