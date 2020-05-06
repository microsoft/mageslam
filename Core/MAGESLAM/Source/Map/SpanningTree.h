// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// SpanningTree.h
//
// the spanning tree contains a spanning tree of the covisibility graph with the edges
// connecting keyframes with the highest number of shared mappoints. it is used as 
// part of the essential graph and is owned by the map.
//------------------------------------------------------------------------------

#pragma once
#include <set>
#include <memory>
#include <Eigen/SparseCore>

namespace mage
{
    class SpanningTree
    {
    public:
        SpanningTree();
        ~SpanningTree();

        // marks this as a valid keyframe for linking to in the spanning tree
        void AddKeyframe(const Id<Keyframe>& keyframeToAdd);

        // fixup spanning tree if keyframes or points have changed
        void UpdateKeyframe(
            const Id<Keyframe>& keyframeToUpdateId,
            const temp::unique_vector<Id<Keyframe>>& curConnectedKeyframes,
            const CovisibilityGraph& covisibility,
            thread_memory memory);

        // indicates this is no longer a valid keyframe fo linking to in the spanning tree
        void RemoveKeyframe(
            const Id<Keyframe>& keyframeToRemove,
            thread_memory memory);

        // checks for cycles and full coverage of covisibility graph
        bool ValidSpanningTree(
            const CovisibilityGraph& covisibility,
            thread_memory memory);

        void DebugOutput() const;

    private:

        bool KeyframeExists(const Id<Keyframe>& keyframeId) const;

        unsigned int GetTreeDistanceFromRoot(const Id<Keyframe>& keyframeId) const;
        
        void UpdateTreeDistanceFromRoot(
            const Id<Keyframe>& keyframeId,
            const Id<Keyframe>& parentId,
            thread_memory memory);

        void AddEdge(const Id<Keyframe>& keyframeId0, const Id<Keyframe>& keyframeId1);
        void RemoveEdge(const Id<Keyframe>& keyframeId0, const Id<Keyframe>& keyframeId1);

        void RepairTreeForRemoval(
            const Id<Keyframe>& keyframeToUpdateId,
            thread_memory memory);

        Id<Keyframe> GetTreeParentKeyframe(const Id<Keyframe>& keyframeId) const;

        void GetTreeChildrenKeyframes(
            const Id<Keyframe>& keyframeId,
            temp::set<Id<Keyframe>>& childrenKeyframes) const;
        
        void GetTreeConnectedKeyframes(
            const Id<Keyframe>& keyframeId,
            temp::set<Id<Keyframe>>& connectedKeyframes) const;

        unsigned int GetEdgeWeight(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>&  keyframe1Id) const;
        void SetEdgeWeight(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>& keyframe1Id, const unsigned int edgeWeight);

        bool ValidSpanningTreeTest(
            const Id<Keyframe>& keyframeFrom,
            const Id<Keyframe>& keyframeToVisit,
            thread_memory memory,
            std::set<Id<Keyframe>>& keyframesVisited);

        Id<Keyframe> m_Root;
        unsigned int m_NumKeyframes;

        Eigen::SparseMatrix<unsigned int, 0, IdT>* mAdjacencyMat;
    };

}