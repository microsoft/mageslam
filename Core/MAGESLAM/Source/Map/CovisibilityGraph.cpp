// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// CovisibilityGraph.h
//
// the covisibility graph relates keyframes that have a significant number of mappoints
// visible in both. it is owned by the map
//------------------------------------------------------------------------------

#include "CovisibilityGraph.h"
#include "Map.h"
#include "MapPoint.h"

#include "MageSettings.h"

#include "Utils\Logging.h"

#include <inttypes.h>
#include <list>

namespace mage
{
    CovisibilityGraph::CovisibilityGraph(const CovisibilitySettings& settings)
        : m_settings{ settings }
    {
        // sparse upper triangular matrix holding the graph. 
        // non-zero entries mean connection between row index keyframe and column index keyframe. 
        // entries are number of mappoints in common.
        mAdjacencyMat = std::make_unique<Eigen::SparseMatrix<unsigned int, 0, IdT>>(INIT_NUM_KEYFRAMES, INIT_NUM_KEYFRAMES);
    }

    CovisibilityGraph::~CovisibilityGraph()
    {}

    /// adds a keyframe to the graph. edges will be added to other keyframes where appropriate
    void CovisibilityGraph::AddKeyframe(const Id<Keyframe>& keyframeId)
    {
        assert(keyframeId.IsValid());

        //mark existance in graph                   
        if (keyframeId.val >= mAdjacencyMat->rows())
        {
            mAdjacencyMat->conservativeResize(keyframeId.val + INIT_NUM_KEYFRAMES, keyframeId.val + INIT_NUM_KEYFRAMES);
        }

        unsigned int& keyframeExists = mAdjacencyMat->insert(keyframeId.val, keyframeId.val);
        assert(keyframeExists == 0); //double add
        keyframeExists = 1;
    }

    void CovisibilityGraph::UpdateKeyframe(
        const Id<Keyframe>& keyframeId,
        const std::map<const Id<Keyframe>, unsigned int>& sharedMapPoints,
        thread_memory memory)
    {
        RemoveAllEdges(keyframeId, memory);
        AddAllEdges(keyframeId, sharedMapPoints);
    }

    /// removes a keyframe from the graph
    void CovisibilityGraph::RemoveKeyframe(
        const Id<Keyframe>& keyframeId,
        thread_memory memory)
    {
        //validate
        assert(KeyframeExists(keyframeId)); //double delete of keyframe?

                                            // all keyframes connected to this one are affected
        RemoveAllEdges(keyframeId, memory);

        //remove existance in graph                    
        unsigned int& keyframeExists = mAdjacencyMat->coeffRef(keyframeId.val, keyframeId.val);
        keyframeExists = 0;

        mAdjacencyMat->prune(0, 0);
    }

    /// adds an edge between two keyframes
    void CovisibilityGraph::AddEdge(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>& keyframe1Id, const unsigned int sharedMapPoints)
    {
        //validate
        unsigned int existingEdgeWeight = GetEdgeWeight(keyframe0Id, keyframe1Id);
        assert(existingEdgeWeight == 0 || existingEdgeWeight == sharedMapPoints); //either this is a new edge, or it is a redundant add

        if (existingEdgeWeight != sharedMapPoints)
        {
            SetEdgeWeight(keyframe0Id, keyframe1Id, sharedMapPoints);
        }
    }

    /// adds all necessary edges for the new keyframe
    void CovisibilityGraph::AddAllEdges(const Id<Keyframe>& keyframeId, const std::map<const Id<Keyframe>, unsigned int>& sharedMapPoints)
    {
        assert(KeyframeExists(keyframeId));

        for (std::map<const Id<Keyframe>, unsigned int>::const_iterator itr = sharedMapPoints.begin(); itr != sharedMapPoints.end(); ++itr)
        {
            assert(itr->first != keyframeId); //self

            if (itr->second >= m_settings.CovisMinThreshold)
            {
                // make entry in covisibility with 2 keyframe id's and a count of shared mappoints
                AddEdge(keyframeId, itr->first, itr->second);
            }
        }
    }

    /// removes all edges from the graph for the specified keyframe
    void CovisibilityGraph::RemoveAllEdges(const Id<Keyframe>& keyframeId, thread_memory memory)
    {
        //validate
        assert(KeyframeExists(keyframeId)); //double delete of keyframe

                                            // TODO PERF probably faster with the ordered_vector/set_vector instead of stack_set
        auto connectedKeyframes = memory.stack_unique_vector<Id<Keyframe>>();
        GetConnectedKeyframes(keyframeId, connectedKeyframes);
        for (const Id<Keyframe>& curKeyframe : connectedKeyframes)
        {
            //validate
            assert(GetEdgeWeight(keyframeId, curKeyframe) != 0);

            //remove edges
            SetEdgeWeight(keyframeId, curKeyframe, 0);
        }
    }

    /// gets the keyframes connected the input keyframe by a number of mappoints above the requested threshold
    ///be sure to pass a clean output set if that's the behavior you want!
    // param theta: the number of mappoints that must be shared in order to be considered connected
    void CovisibilityGraph::GetConnectedKeyframes(
        const Id<Keyframe>& keyframeId,
        temp::unique_vector<Id<Keyframe>>& connectedKeyframeIds,
        unsigned int theta) const
    {
        //validate
        assert(KeyframeExists(keyframeId));

        // it's an upper triangular matrix, all the numbers less than you are in your column 
        // and all the numbers greater are in your row


        //check entries with ids < keyframes's id (column)                 
        for (Eigen::SparseMatrix<unsigned int, 0, IdT>::InnerIterator rowIt(*mAdjacencyMat, keyframeId.val); rowIt; ++rowIt)
        {
            //skip self
            if (rowIt.col() == rowIt.row())
                continue;

            unsigned int edgeMapPoints = rowIt.value();
            if (edgeMapPoints >= theta)
            {
                assert(rowIt.row() != keyframeId.val);
                connectedKeyframeIds.insert<Id<Keyframe>>(rowIt.row());
            }
        }

        //check entries with ids > keyframes's id (row)                           
        const IdT numCols = mAdjacencyMat->outerSize();
        for (IdT col = keyframeId.val + 1; col < numCols; ++col)
        {
            unsigned int edgeMapPoints = mAdjacencyMat->coeff(keyframeId.val, col);
            if (edgeMapPoints >= theta)
            {
                connectedKeyframeIds.insert<Id<Keyframe>>(col);
            }
        }
    }

    void CovisibilityGraph::GetConnectedSubGraphs(loop::set<Id<Keyframe>>& goodIds, thread_memory memory, std::vector<std::vector<Id<Keyframe>>>& output) const
    {
        auto touched = memory.stack_set<Id<Keyframe>>();
        auto currentCovis = memory.stack_unique_vector<Id<Keyframe>>();
        while (goodIds.size() > 0)
        {
            touched.emplace(*goodIds.begin());
            goodIds.erase(goodIds.begin());

            output.emplace_back();
            auto& covisibleGroup = output.back();

            while (touched.size() > 0)
            {
                auto pCurr = touched.begin();
                covisibleGroup.emplace_back(*pCurr);

                currentCovis.clear();
                GetConnectedKeyframes(*pCurr, currentCovis);

                for (const auto id : currentCovis)
                {
                    // If the covisible ID is in goodIds, then it certainly hasn't been touched yet.
                    auto found = goodIds.find(id);
                    if (found != goodIds.end())
                    {
                        touched.emplace(*found);
                        goodIds.erase(found);
                    }
                }

                touched.erase(pCurr);
            }
        }
    }

    // gets the set of keyframes that have valid edges in the covisibility graph (debugging helper)
    // this function will be slow
    std::set<Id<Keyframe>> CovisibilityGraph::GetAllCovisibilityConnectedKeyframes() const
    {
        std::set<Id<Keyframe>> outSet;

        const IdT numCols = mAdjacencyMat->outerSize();

        for (IdT col = 0; col < numCols; ++col)
        {
            for (Eigen::SparseMatrix<unsigned int, 0, IdT>::InnerIterator itRow(*mAdjacencyMat, col); itRow; ++itRow)
            {
                if (itRow.row() == itRow.col())
                    continue;

                if (itRow.value() == 0)
                    continue;

                outSet.insert(itRow.row());
                outSet.insert(itRow.col());
            }
        }
        return outSet;
    }

    unsigned int CovisibilityGraph::GetEdgeWeight(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>& keyframe1Id) const
    {
        //validate
        assert(KeyframeExists(keyframe0Id));
        assert(KeyframeExists(keyframe1Id));
        assert(keyframe0Id != keyframe1Id);

        //find entry in adjacency matrix      
        if (keyframe0Id < keyframe1Id)
        {
            unsigned int edgeMapPoints = mAdjacencyMat->coeff(keyframe0Id.val, keyframe1Id.val);
            return edgeMapPoints;
        }
        else
        {
            unsigned int edgeMapPoints = mAdjacencyMat->coeff(keyframe1Id.val, keyframe0Id.val);
            return edgeMapPoints;
        }
    }

    void CovisibilityGraph::SetEdgeWeight(const Id<Keyframe>& keyframe0Id, const Id<Keyframe>& keyframe1Id, const unsigned int edgeWeight)
    {
        //validate
        assert(KeyframeExists(keyframe0Id));
        assert(KeyframeExists(keyframe1Id));
        assert(keyframe0Id != keyframe1Id);

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

    const Id<Keyframe> CovisibilityGraph::GetKeyframeWithMostSharedMapPoints(
        const Id<Keyframe>& keyframeIdToMatch,
        const temp::unique_vector<Id<Keyframe>>& connectedKeyframes,
        const Id<Keyframe>& excludingKeyframeId,
        unsigned int& numMatchedMapPoints) const
    {
        Id<Keyframe> mostMatchesId;
        for (const Id<Keyframe>& curKeyframeId : connectedKeyframes)
        {
            if (curKeyframeId == excludingKeyframeId)
                continue;

            unsigned int curMatchedMapPoints = GetEdgeWeight(keyframeIdToMatch, curKeyframeId);
            if (curMatchedMapPoints > numMatchedMapPoints)
            {
                numMatchedMapPoints = curMatchedMapPoints;
                mostMatchesId = curKeyframeId;
            }
        }

        //assert(mostMatches > 0);    //keyframe exists in graph, but didn't have enough map points in common for a covis edge
        assert(mostMatchesId != keyframeIdToMatch);

        return mostMatchesId;
    }

    unsigned int CovisibilityGraph::GetNumSharedMapPoints(const Id<Keyframe>& keyframeId0, const Id<Keyframe>& keyframeId1) const
    {
        return GetEdgeWeight(keyframeId0, keyframeId1);
    }

    const Id<Keyframe> CovisibilityGraph::GetKeyframeWithMostSharedMapPoints(
        const Id<Keyframe>& keyframeIdToMatch,
        const Id<Keyframe>& excludingKeyframeId,
        thread_memory memory,
        unsigned int& numMatchedMapPoints) const
    {
        assert(KeyframeExists(keyframeIdToMatch));

        // search for most matched mappoints
        numMatchedMapPoints = 0;
        auto connectedKeyframes = memory.stack_unique_vector<Id<Keyframe>>();
        GetConnectedKeyframes(keyframeIdToMatch, connectedKeyframes);

        return GetKeyframeWithMostSharedMapPoints(keyframeIdToMatch, connectedKeyframes, excludingKeyframeId, numMatchedMapPoints);
    }

    // if a keyframe is a known keyframe a 1 is tucked into the graph on the diagonal (id,id)
    bool CovisibilityGraph::KeyframeExists(const Id<Keyframe>& keyframeId) const
    {
        return 1 == mAdjacencyMat->coeff(keyframeId.val, keyframeId.val);
    }

#pragma optimize("",off)
    void CovisibilityGraph::DebugOutput() const
    {
        std::wstringstream ss;
        ss << "----Begin Covisibility Graph----" << std::endl;

        const IdT numCols = mAdjacencyMat->outerSize();

        for (IdT col = 0; col < numCols; ++col)
        {
            for (Eigen::SparseMatrix<unsigned int, 0, IdT>::InnerIterator itRow(*mAdjacencyMat, col); itRow; ++itRow)
            {
                if (itRow.row() == itRow.col())
                    continue;

                if (itRow.value() == 0)
                    continue;

                ss << "Edge: " << itRow.row() << " <=> " << itRow.col() << " (" << itRow.value() << ")" << std::endl;
            }
        }

        ss << "----End Covisibility Tree----" << std::endl;

        LogMessage<>(ss.str());
    }
#pragma optimize("",on)
}