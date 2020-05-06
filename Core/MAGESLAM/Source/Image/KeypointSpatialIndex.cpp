// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// KeypointSpatialIndex.cpp
//
// Allows to construction, query, and removal of Keypoint entries based on pixel position.
//------------------------------------------------------------------------------

#include "KeypointSpatialIndex.h"

#include <boost\function_output_iterator.hpp>
// TODO:  feed back to Boost that 4457 is firing
#pragma warning (push, 1)
#pragma warning (disable:4457)
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost\geometry\index\rtree.hpp>
#pragma warning (pop)

using namespace cv;
using namespace std;

namespace mage
{
    // TODO PERF optimize this.  Box queries seem to typically return about 16 hits.
    using indexType = boost::geometry::index::rstar<12>;
    using rtree_point = boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>;
    using rtree_box = boost::geometry::model::box<rtree_point>;
    using rtree_value = tuple<rtree_point, size_t>;
    using rtree = boost::geometry::index::rtree<rtree_value, indexType>;
    
    struct KeypointSpatialIndex::Impl
    {
        rtree index;

        Impl(gsl::span<const rtree_value> values);
    };

    KeypointSpatialIndex::Impl::Impl(gsl::span<const rtree_value> values) :
        index(values.begin(), values.end())
    {
    }


    KeypointSpatialIndex::KeypointSpatialIndex(gsl::span<const cv::KeyPoint> keypoints)
    {
        // TODO PERF and cleanliness:  use range adapters to create.
        // http://www.boost.org/doc/libs/1_58_0/libs/geometry/doc/html/geometry/spatial_indexes/rtree_examples/range_adaptors.html
        vector<rtree_value> valuesStruct;
        valuesStruct.reserve(keypoints.size());

        for (ptrdiff_t i = 0; i < keypoints.size(); ++i)
        {
            valuesStruct.emplace_back(rtree_point(keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].octave * octaveSpacing), i);
        }

        m_impl = std::make_unique<KeypointSpatialIndex::Impl>(valuesStruct);
    }

    KeypointSpatialIndex::~KeypointSpatialIndex()
    {}

    struct index_emplace_back
    {
        index_emplace_back& operator =(const rtree_value& value)
        {
            results.emplace_back(get<1>(value));
            return *this;
        }

        index_emplace_back& operator *()
        {
            return *this;
        }

        index_emplace_back& operator++()
        {
            return *this;
        }

        index_emplace_back(temp::vector<size_t>& output)
            : results{ output }
        {}

        temp::vector<size_t>& results;
    };

    void KeypointSpatialIndex::Query(const cv::Point2f& center, int octave, float radius, temp::vector<size_t>& results) const
    {
        rtree_box queryBox{
            rtree_point{ center.x - radius, center.y - radius, octave * octaveSpacing - octaveQueryRange },
            rtree_point{ center.x + radius, center.y + radius, octave * octaveSpacing + octaveQueryRange }
        };

        m_impl->index.query(boost::geometry::index::intersects(queryBox), index_emplace_back{ results });
    }


    void KeypointSpatialIndex::Remove(const cv::KeyPoint& keypoint, size_t value)
    {
        size_t elem = m_impl->index.remove(
            rtree_value(rtree_point(keypoint.pt.x, keypoint.pt.y, keypoint.octave * octaveSpacing), value));
        (void)elem;
        assert(elem == 1);
    }
}