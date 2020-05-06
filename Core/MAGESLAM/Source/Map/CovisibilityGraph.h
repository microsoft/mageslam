// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// CovisibilityGraph.h
//
// the covisibility graph relates keyframes that have a significant number of mappoints
// visible in both. it is owned by the map
//------------------------------------------------------------------------------

#pragma once

#include "MageSettings.h"
#include "Data\Keyframe.h"

#include <vector>
#include <memory>
#include <set>

#include <Eigen/SparseCore>

namespace mage
{
    class CovisibilityGraph
    {
    public:
        CovisibilityGraph(const CovisibilitySettings&);
        ~CovisibilityGraph();

        CovisibilityGraph(CovisibilityGraph&&) = default;

        // tells the graph this is a valid keyframe for building links with
        void AddKeyframe(const Id<Keyframe>& keyframeId);

        // builds/removes links based on associations currently held by the keyframe
        void UpdateKeyframe(
            const Id<Keyframe>& keyframeId,
            const std::map<const Id<Keyframe>, unsigned int>& sharedMapPoints,
            thread_memory memory);

        // tells the graph this is no longeer a valid keyframe for building links with
        void RemoveKeyframe(
            const Id<Keyframe>& keyframe,
            thread_memory memory);

        const Id<Keyframe> GetKeyframeWithMostSharedMapPoints(
            const Id<Keyframe>& keyframeToMatch,
            const Id<Keyframe>& excluding,
            thread_memory memory,
            unsigned int& numMatchedMapPoints) const;

        const Id<Keyframe> GetKeyframeWithMostSharedMapPoints(
            const Id<Keyframe>& keyframeIdToMatch,
            const temp::unique_vector<Id<Keyframe>>& connectedKeyframes,
            const Id<Keyframe>& excludingKeyframeId,
            unsigned int& numMatchedMapPoints) const;

        unsigned int GetNumSharedMapPoints(const Id<Keyframe>& keyframeId0, const Id<Keyframe>& keyframeId1) const;

        void GetConnectedKeyframes(
            const Id<Keyframe>& keyframeId,
            temp::unique_vector<Id<Keyframe>>& connectedKeyframes,
            unsigned int theta) const;

        void GetConnectedKeyframes(
            const Id<Keyframe>& keyframeId,
            temp::unique_vector<Id<Keyframe>>& connectedKeyframes) const
        {
            GetConnectedKeyframes(keyframeId, connectedKeyframes, m_settings.CovisMinThreshold);
        }

        // Collect contiguous sets.
        //
        // Uses a "loose" notion of a covisible cluster, which avoids ambiguities that would
        // lead to a single keyframe appearing in multiple clusters.  For example, given
        // the similar set {A,B,C,D} with covisible pairs {AB,AC,BC,BD,CD}, a "tight" notion
        // of a covisible cluster would require this to be reported as two clusters, {A,B,C} 
        // and {B,C,D}, with significant overlap among them.  By contrast, a "loose" notion
        // reports this as just the one cluster {A,B,C,D}, even though AD is not a covisible
        // pair.  So, a "tight" covisible cluster is one in which every keyframe in the cluster
        // is covisible with EVERY other keyframe in the cluster; whereas a "loose" covisible
        // cluster merely contains a connected covisibility graph.
        // 
        // Consequently, the clusters returned by this method are simply all the connected
        // covisibility subgraphs that can be built from the similar keyframes.  The following
        // code just uses a BFS approach to find these connected subgraphs.
        void GetConnectedSubGraphs(loop::set<Id<Keyframe>>&, thread_memory, std::vector<std::vector<Id<Keyframe>>>& output) const;

        // gets the set of keyframes that have valid edges in the covisibility graph (debugging helper)
        std::set<Id<Keyframe>> GetAllCovisibilityConnectedKeyframes() const;

        const CovisibilitySettings& GetSettings() const { return m_settings; }

        void DebugOutput() const;

    private:
        void AddEdge(const Id<Keyframe>& keyframe0, const Id<Keyframe>& keyframe1, const unsigned int sharedMapPoints);
        void AddAllEdges(const Id<Keyframe>& keyframeId, const std::map<const Id<Keyframe>, unsigned int>& sharedMapPoints);
        void RemoveAllEdges(const Id<Keyframe>& keyframe, thread_memory memory);

        bool KeyframeExists(const Id<Keyframe>& keyframe) const;

        unsigned int GetEdgeWeight(const Id<Keyframe>&  keyframe0, const Id<Keyframe>& keyframe1) const;
        void SetEdgeWeight(const Id<Keyframe>& keyframe0, const Id<Keyframe>& keyframe1, const unsigned int edgeWeight);

        std::unique_ptr<Eigen::SparseMatrix<unsigned int, 0, IdT>> mAdjacencyMat;

        const CovisibilitySettings& m_settings;
    };
}
