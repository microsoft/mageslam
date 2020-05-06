// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// SpanningTree.h
//
// the spanning tree holds links between keyframes that share the most mappoints
//------------------------------------------------------------------------------

#include "assert.h"
#include "Map.h"
#include "SpanningTree.h"
#include "CovisibilityGraph.h"

#include "Data\Keyframe.h"

#include "MageSettings.h"

#include "Utils\Logging.h"

using namespace std;

namespace mage
{

    SpanningTree::SpanningTree()
        : m_NumKeyframes(0)
    {
        // sparse upper triangular matrix holding the graph. 
        // non-zero entries mean connection between row index keyframe and column index keyframe. 
        // entries are numMatches or 0 (connection or not)
        // entry on diagonal means its a known keyframe, value is the distance from root
        mAdjacencyMat = new Eigen::SparseMatrix<unsigned int, 0, IdT>(INIT_NUM_KEYFRAMES, INIT_NUM_KEYFRAMES);
    }

    SpanningTree::~SpanningTree()
    {
        delete mAdjacencyMat;
    }

    // add a keyframe to the graph (indicates a valid node only, not connected)
    void SpanningTree::AddKeyframe(const Id<Keyframe>& keyframeToAddId)
    {
        assert(!KeyframeExists(keyframeToAddId));

        if (keyframeToAddId.val >= mAdjacencyMat->rows())
        {
            mAdjacencyMat->conservativeResize(keyframeToAddId.val + INIT_NUM_KEYFRAMES, keyframeToAddId.val + INIT_NUM_KEYFRAMES);
        }

        //mark existance in graph                   
        unsigned int& keyframeExists = mAdjacencyMat->insert(keyframeToAddId.val, keyframeToAddId.val);
        assert(keyframeExists == 0); //double add
        keyframeExists = 1;
        m_NumKeyframes++;
    }

    // remove a keyframe from the spanning tree
    void SpanningTree::RemoveKeyframe(
        const Id<Keyframe>& keyframeToRemoveId,
        thread_memory memory)
    {
        assert(KeyframeExists(keyframeToRemoveId));

        //ensure spanning tree remains connected
        RepairTreeForRemoval(keyframeToRemoveId, memory);

        //remove keyframe
        unsigned int& keyframeExists = mAdjacencyMat->coeffRef(keyframeToRemoveId.val, keyframeToRemoveId.val);
        keyframeExists = 0;
        assert(m_NumKeyframes > 0);
        m_NumKeyframes--;

        // Minimal-impact "fix" for the problem that our sparse matrices get filled with explicit zeros.
        mAdjacencyMat->prune(0, 0);
    }

    // based on interpretation of the paper, the spanning tree doesn't update its links to constantly point to the most connected
    // keyframe. it does so during add only. the other case it needs to handle is a keyframe removal
    void SpanningTree::UpdateKeyframe(
        const Id<Keyframe>& keyframeToUpdateId,
        const temp::unique_vector<Id<Keyframe>>& curConnectedKeyframes,
        const CovisibilityGraph& covisibility,
        thread_memory memory)
    {
        //get existing state
        temp::set<Id<Keyframe>> prevConnectedKeyframes = memory.stack_set<Id<Keyframe>>();
        GetTreeConnectedKeyframes(keyframeToUpdateId, prevConnectedKeyframes);

        if ((prevConnectedKeyframes.size() == 0) && (curConnectedKeyframes.size() > 0))
        {
            //add self to spanning tree

            Id<Keyframe> noExclude;
            unsigned int newNumMatches = 0;
            Id<Keyframe> curMostConnectedKeyframeId = covisibility.GetKeyframeWithMostSharedMapPoints(keyframeToUpdateId, curConnectedKeyframes, noExclude, newNumMatches);

            if (!m_Root.IsValid())
            {
                m_Root = keyframeToUpdateId;
                UpdateTreeDistanceFromRoot(keyframeToUpdateId, {}, memory);
            }

            if (curMostConnectedKeyframeId.IsValid())
            {
                AddEdge(keyframeToUpdateId, curMostConnectedKeyframeId);

                if (m_Root == keyframeToUpdateId)
                {
                    UpdateTreeDistanceFromRoot(curMostConnectedKeyframeId, m_Root, memory);
                }
                else
                {
                    UpdateTreeDistanceFromRoot(keyframeToUpdateId, curMostConnectedKeyframeId, memory);
                }
            }
        }
        else if ((prevConnectedKeyframes.size() > 0) && (curConnectedKeyframes.size() == 0))
        {
            //remove self from spanning tree            
            RepairTreeForRemoval(keyframeToUpdateId, memory);
        }
    }

    void SpanningTree::RepairTreeForRemoval(
        const Id<Keyframe>& keyframeToUpdateId,
        thread_memory memory)
    {
        temp::set<Id<Keyframe>> prevConnectedKeyframes = memory.stack_set<Id<Keyframe>>();
        GetTreeConnectedKeyframes(keyframeToUpdateId, prevConnectedKeyframes);

        temp::set<Id<Keyframe>> prevConnectedChildren = memory.stack_set<Id<Keyframe>>();
        GetTreeChildrenKeyframes(keyframeToUpdateId, prevConnectedChildren);
        Id<Keyframe> prevConnectedParent = GetTreeParentKeyframe(keyframeToUpdateId);

        // remove links to connected keyframes
        for (const Id<Keyframe>& keyframeId : prevConnectedKeyframes)
        {
            RemoveEdge(keyframeToUpdateId, keyframeId);
        }

        // update root if needed
        if (keyframeToUpdateId == m_Root)
        {
            if (prevConnectedKeyframes.size() == 0)
            {
                m_Root = Id<Keyframe>();
            }
            else
            {
                m_Root = *prevConnectedKeyframes.begin();
                UpdateTreeDistanceFromRoot(m_Root, {}, memory);
            }
        }

        //connect orphaned keyframes to ancestor
        //make link
        if (prevConnectedParent.IsValid())
        {
            for (const Id<Keyframe>& childKeyframeId : prevConnectedChildren)
            {
                AddEdge(childKeyframeId, prevConnectedParent);
                UpdateTreeDistanceFromRoot(childKeyframeId, prevConnectedParent, memory);
            }
        }
    }

    void SpanningTree::GetTreeConnectedKeyframes(
        const Id<Keyframe>& keyframeId,
        temp::set<Id<Keyframe>>& connectedKeyframes) const
    {
        //validate
        assert(KeyframeExists(keyframeId));

        for (Eigen::SparseMatrix<unsigned int, 0, IdT>::InnerIterator rowIt(*mAdjacencyMat, keyframeId.val); rowIt; ++rowIt)
        {
            //skip self
            if (rowIt.col() == rowIt.row())
                continue;

            unsigned int edgeExists = rowIt.value();
            if (edgeExists > 0)
            {
                connectedKeyframes.insert(rowIt.row());
            }
        }

        //check entries with ids > keyframes's id (column)
        const IdT numKeyframes = mAdjacencyMat->outerSize();
        for (IdT col = keyframeId.val + 1; col < numKeyframes; ++col)
        {
            unsigned int edgeExists = mAdjacencyMat->coeff(keyframeId.val, col);
            if (edgeExists > 0)
            {
                connectedKeyframes.insert(col);
            }
        }
    }

    Id<Keyframe> SpanningTree::GetTreeParentKeyframe(const Id<Keyframe>& keyframeId) const
    {
        //validate
        assert(KeyframeExists(keyframeId));

        Id<Keyframe> parent;

        //get self distance from root
        unsigned int keyframeDist = GetTreeDistanceFromRoot(keyframeId);

        for (Eigen::SparseMatrix<unsigned int, 0, IdT>::InnerIterator rowIt(*mAdjacencyMat, keyframeId.val); rowIt; ++rowIt)
        {
            //skip self
            if (rowIt.col() == rowIt.row())
                continue;

            if (rowIt.value() > 0)
            {
                unsigned int curKfDist = GetTreeDistanceFromRoot(rowIt.row());
                if (curKfDist < keyframeDist)
                {
                    assert(false == parent.IsValid());  //each keyframe should have a single parent 
                    parent = rowIt.row();               //PERF: once correctness established, can return here
                }
            }
        }

        //check entries with ids > keyframes's id (column)
        const IdT numKeyframes = mAdjacencyMat->outerSize();
        for (IdT col = keyframeId.val + 1; col < numKeyframes; ++col)
        {
            if (mAdjacencyMat->coeff(keyframeId.val, col) > 0)
            {
                unsigned int curKfDist = GetTreeDistanceFromRoot(col);
                if (curKfDist < keyframeDist)
                {
                    assert(false == parent.IsValid()); //each keyframe should have a single parent 
                    parent = col;                       //PERF: once correctness established, can return here
                }
            }
        }

        return parent;
    }

    void SpanningTree::GetTreeChildrenKeyframes(
        const Id<Keyframe>& keyframeId,
        temp::set<Id<Keyframe>>& childrenKeyframes) const
    {
        //validate
        assert(KeyframeExists(keyframeId));

        //get self distance from root
        unsigned int keyframeDist = GetTreeDistanceFromRoot(keyframeId);

        for (Eigen::SparseMatrix<unsigned int, 0, IdT>::InnerIterator rowIt(*mAdjacencyMat, keyframeId.val); rowIt; ++rowIt)
        {
            //skip self
            if (rowIt.col() == rowIt.row())
                continue;

            if (rowIt.value() > 0)
            {
                unsigned int curKfDist = GetTreeDistanceFromRoot(rowIt.row());
                if (curKfDist > keyframeDist)
                {
                    childrenKeyframes.insert(rowIt.row());
                }
            }
        }

        //check entries with ids > keyframes's id (column)
        const IdT numKeyframes = mAdjacencyMat->outerSize();
        for (IdT col = keyframeId.val + 1; col < numKeyframes; ++col)
        {
            if (mAdjacencyMat->coeff(keyframeId.val, col) > 0)
            {
                unsigned int curKfDist = GetTreeDistanceFromRoot(col);
                if (curKfDist > keyframeDist)
                {
                    childrenKeyframes.insert(col);
                }
            }
        }
    }

#pragma optimize("",off)
    void SpanningTree::DebugOutput() const
    {
        wstringstream ss;
        ss << "----Begin Spanning Tree----" << std::endl;

        if (!m_Root.IsValid())
        {
            ss << "Empty Tree" << std::endl;
        }
        else
        {
            unsigned int rootDist = GetEdgeWeight(m_Root, m_Root);

            ss << "Root: " << m_Root.val << " (" << rootDist << ")" << std::endl;

            for (int k = 0; k < mAdjacencyMat->outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<unsigned int>::InnerIterator it(*mAdjacencyMat, k); it; ++it)
                {
                    if (it.row() == it.col())
                        continue;

                    if (it.value() == 0)
                        continue;

                    unsigned int rowDist = GetEdgeWeight(it.row(), it.row());
                    unsigned int colDist = GetEdgeWeight(it.col(), it.col());
                    ss << "Edge: (" << rowDist << ") " << it.row() << " <=> " << it.col() << " (" <<colDist << ") : (" << it.value() << ")" << std::endl;
                }
            }
        }
        ss << "----End Spanning Tree----" << std::endl;

        LogMessage<>(ss.str());
    }
#pragma optimize("",on)

    // PERF: exhaustive search, very slow only for debugging    
    bool SpanningTree::ValidSpanningTreeTest(
        const Id<Keyframe>& keyframeFrom,
        const Id<Keyframe>& curKeyframe,
        thread_memory memory,
        std::set<Id<Keyframe>>& keyframesVisited)
    {
        if (keyframesVisited.end() != std::find(keyframesVisited.begin(), keyframesVisited.end(), curKeyframe))
            return false; //detected a cycle        

        //verify ordering is consistent
        if (keyframeFrom.IsValid())
        {
            unsigned int parentDistance = GetEdgeWeight(keyframeFrom, keyframeFrom);
            unsigned int curDistance = GetEdgeWeight(curKeyframe, curKeyframe);
            if (curDistance != parentDistance + 1)
                return false;
        }

        //add to visited
        keyframesVisited.insert(curKeyframe);

        // add childrent to list to visit        
        temp::set<Id<Keyframe>> keyframesConnected = memory.stack_set<Id<Keyframe>>();
        GetTreeConnectedKeyframes(curKeyframe, keyframesConnected);
        for (Id<Keyframe> curConnectedKeyframeId : keyframesConnected)
        {
            if (curConnectedKeyframeId == keyframeFrom)
                continue;

            if (false == ValidSpanningTreeTest(curKeyframe, curConnectedKeyframeId, memory, keyframesVisited))
                return false;
        }

        return true;
    }

    bool SpanningTree::ValidSpanningTree(
        const CovisibilityGraph& covisibility,
        thread_memory memory)
    {
        //every node in the covisibility graph that has a link should be present and visited by a walk of the spanning tree
        set<Id<Keyframe>> covisKeyframes = covisibility.GetAllCovisibilityConnectedKeyframes();
        if (covisKeyframes.size() > 0)
        {
            if (!m_Root.IsValid())
            {
                // TODO: Tracing OutputDebugStringA("Bad root");
                covisibility.DebugOutput();
                DebugOutput();
                return false;
            }

            //walk spanning tree for cycles and ensure all covisibilty connected nodes were visited
            std::set<Id<Keyframe>> keyframesVisited;
            if (false == ValidSpanningTreeTest({}, m_Root, memory, keyframesVisited))
            {
                // TODO: Tracing OutputDebugStringA("Cycle found");
                covisibility.DebugOutput();
                DebugOutput();
                return false;
            }

            if (keyframesVisited.size() != covisKeyframes.size())
            {
                // TODO: Tracing OutputDebugStringA("Bad count");
                covisibility.DebugOutput();
                DebugOutput();
                return false;
            }

            for (Id<Keyframe> curKeyframe : covisKeyframes)
            {
                if (keyframesVisited.end() == keyframesVisited.find(curKeyframe))
                {
                    // TODO: Tracing OutputDebugStringA("Didn't visit all nodes");
                    covisibility.DebugOutput();
                    DebugOutput();
                    return false;
                }
            }
        }

        return true;
    }

    unsigned int SpanningTree::GetEdgeWeight(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>& keyframe1Id) const
    {
        //validate
        assert(KeyframeExists(keyframe0Id));
        assert(KeyframeExists(keyframe1Id));

        //find entry in adjacency matrix
        if (keyframe0Id < keyframe1Id)
        {
            return mAdjacencyMat->coeff(keyframe0Id.val, keyframe1Id.val);
        }
        else
        {
            return mAdjacencyMat->coeff(keyframe1Id.val, keyframe0Id.val);
        }
    }

    void SpanningTree::SetEdgeWeight(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>& keyframe1Id, const unsigned int edgeWeight)
    {
        //validate
        assert(keyframe0Id.IsValid());
        assert(keyframe1Id.IsValid());
        assert(KeyframeExists(keyframe0Id));
        assert(KeyframeExists(keyframe1Id));

        if (keyframe0Id < keyframe1Id)
        {
            unsigned int& edgeMapPoints = mAdjacencyMat->coeffRef(keyframe0Id.val, keyframe1Id.val);
            edgeMapPoints = edgeWeight;
        }
        else
        {
            unsigned int& edgeMapPoints = mAdjacencyMat->coeffRef(keyframe1Id.val, keyframe0Id.val);
            edgeMapPoints = edgeWeight;
        }
    }

    // if a keyframe is a known keyframe a 1 is tucked into the graph on the diagonal (id,id)
    bool SpanningTree::KeyframeExists(const Id<Keyframe>& keyframeId) const
    {
        assert(keyframeId.IsValid());

        if (keyframeId.val >= mAdjacencyMat->rows() || keyframeId.val >= mAdjacencyMat->cols())
            return false;

        return 0 != mAdjacencyMat->coeff(keyframeId.val, keyframeId.val);
    }

    unsigned int SpanningTree::GetTreeDistanceFromRoot(const Id<Keyframe>& keyframeId) const
    {
        return  GetEdgeWeight(keyframeId, keyframeId);
    }

    //update new node's distance from root (one based, root: 1, 0 means keyframe is unknown)
    void SpanningTree::UpdateTreeDistanceFromRoot(
        const Id<Keyframe>& keyframeId,
        const Id<Keyframe>& parentId,
        thread_memory memory)
    {
        if (!parentId.IsValid() || keyframeId == m_Root)
        {
            assert(keyframeId == m_Root);
            SetEdgeWeight(keyframeId, keyframeId, 1);
        }
        else
        {
            unsigned int parentDistance = GetTreeDistanceFromRoot(parentId);
            SetEdgeWeight(keyframeId, keyframeId, parentDistance + 1);
        }

        temp::set<Id<Keyframe>> connectedKeyframes = memory.stack_set<Id<Keyframe>>();
        GetTreeConnectedKeyframes(keyframeId, connectedKeyframes);
        
        for (const Id<Keyframe>& curConnectedId : connectedKeyframes)
        {
            if (curConnectedId == parentId)
                continue;

            UpdateTreeDistanceFromRoot(curConnectedId, keyframeId, memory);
        }

    }

    void SpanningTree::AddEdge(const Id<Keyframe>& keyframeId0, const Id<Keyframe>& keyframeId1)
    {
        SetEdgeWeight(keyframeId0, keyframeId1, 1);
    }

    void SpanningTree::RemoveEdge(const Id<Keyframe>& keyframeId0, const Id<Keyframe>& keyframeId1)
    {
        SetEdgeWeight(keyframeId0, keyframeId1, 0);
    }


}