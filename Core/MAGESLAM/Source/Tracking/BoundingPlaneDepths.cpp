// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "BoundingPlaneDepths.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "Utils\cv.h"
#include "Utils\Logging.h"
#include "Reprojection.h"

namespace mage
{
    InternalDepth CalculateBoundingPlaneDepthsForKeyframe(const KeyframeProxy& keyframe, const BoundingDepthSettings& settings)
    {
        SCOPE_TIMER(CalculateBoundingPlaneDepthsForKeyframe);

        std::vector<MapPointTrackingProxy> points;
        keyframe.GetMapPoints(points);

        InternalDepth depthStruct{ std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), points.size() };

        cv::Vec3f f = keyframe.GetPose().GetWorldSpaceForward();
        cv::Point3f c = keyframe.GetPose().GetWorldSpacePosition();

        assert(abs(f.dot(f) - 1.0f) < NRM_EPSILON); // If the forward vector isn't a unit vector, we have big problems.
        
        std::vector<float> validDepths;

        const cv::Matx34f& viewMat = keyframe.GetPose().GetViewMatrix();
        const cv::Matx33f& camMat = keyframe.GetAnalyzedImage()->GetUndistortedCalibration().GetCameraMatrix();

        const float minX = settings.RegionOfInterestMinX * keyframe.GetAnalyzedImage()->GetWidth();
        const float minY = settings.RegionOfInterestMinY * keyframe.GetAnalyzedImage()->GetHeight();
        const float maxX = settings.RegionOfInterestMaxX * keyframe.GetAnalyzedImage()->GetWidth();
        const float maxY = settings.RegionOfInterestMaxY * keyframe.GetAnalyzedImage()->GetHeight();

        std::unique_ptr<ProjectedPoint[]> projectedPoints{ new ProjectedPoint[points.size()] };
        for (size_t idx = 0; idx < points.size(); idx++)
        {
            // Only consider map points whose keypoints are within the region of interest.
            cv::KeyPoint kpt = keyframe.GetAssociatedKeyPoint(points[idx]);
            if (kpt.pt.x >= minX && kpt.pt.x <= maxX &&
                kpt.pt.y >= minY && kpt.pt.y <= maxY)
            {
                float depth = (points[idx].GetPosition() - c).dot(f);
                assert(depth > 0);

                validDepths.push_back(depth);
            }

            // project the points into the current frame in order to provide a sparse depth "map"
            Projection projected = ProjectUndistorted(viewMat, camMat, points[idx].GetPosition());
            projectedPoints[idx].X = projected.Point.x;
            projectedPoints[idx].Y = projected.Point.y;
            projectedPoints[idx].Depth = projected.Distance;
            projectedPoints[idx].Id = reinterpret_cast<const Id<MapPoint>::storage_t&>(points[idx].GetId());
        }

        depthStruct.SparseDepth = std::move(projectedPoints);

        // Applies the "softness" settings, which rule out a certain percentage of the
        // discovered depths at the respective extremes (near and far).
        if (validDepths.size() > 0)
        {
            size_t softNearIdx = (size_t)(settings.NearDepthSoftness * validDepths.size());
            std::nth_element(validDepths.begin(), validDepths.begin() + softNearIdx, validDepths.end());
            depthStruct.NearPlaneDepth = validDepths[softNearIdx];

            size_t softFarReverseIdx = (size_t)(settings.FarDepthSoftness * validDepths.size());
            std::nth_element(validDepths.begin(), validDepths.begin() + softFarReverseIdx, validDepths.end(), std::greater<float>{});
            depthStruct.FarPlaneDepth = validDepths[softFarReverseIdx];
        }

        // This is possible if there are no map points whose associated keypoints are
        // within the region of interest.  If this happens, the depth calculation is
        // invalid.
        if (depthStruct.NearPlaneDepth > depthStruct.FarPlaneDepth)
        {
            depthStruct.NearPlaneDepth = Depth::INVALID_DEPTH;
            depthStruct.FarPlaneDepth = Depth::INVALID_DEPTH;
        }

        return depthStruct;
    }
}
