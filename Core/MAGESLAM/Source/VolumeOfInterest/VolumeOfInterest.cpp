// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "VolumeOfInterest.h"
#include "Debugging/SkeletonLogger.h"
#include "Map/PoseHistory.h"
#include <arcana/math.h>
#include <arcana/analysis/determinator.h>
#include <math.h>

using namespace std;

namespace mage
{
    struct VoxelSpace
    {
        cv::Vec3i dimensions;
        vector<float>voxels;

        VoxelSpace(const cv::Vec3f dims)
            : dimensions{ (int)ceil(dims[0]), (int)ceil(dims[1]), (int)ceil(dims[2]) },
            voxels(dimensions[0] * dimensions[1] * dimensions[2])
        {
        }

        float GetVoxel(int x, int y, int z) const
        {
            return voxels[Idx(x, y, z)];
        }

        void SetVoxel(int x, int y, int z, float value)
        {
            voxels[Idx(x, y, z)] = value;
        }

    private:
        int Idx(int x, int y, int z) const
        {
            return x +
                dimensions[0] * y +
                dimensions[0] * dimensions[1] * z;
        }
    };

    VOIKeyframe::VOIKeyframe(const Pose& pose, const Depth& depth, const VolumeOfInterestSettings& voiSettings)
        : VoiSettings{ voiSettings }, WorldPosition{ pose.GetWorldSpacePosition() }, NearDepth{ depth.NearPlaneDepth }, FarDepth{ depth.FarPlaneDepth }
    {
        cv::Matx33f rotationMat = Rotation(pose.GetInverseViewMatrix()) * ToMat(VoiSettings.KernelPitchRads, VoiSettings.KernelRollRads, VoiSettings.KernelYawRads);
        cv::Matx31f forwardMat = rotationMat.col(2);
        Forward = { forwardMat.val[0], forwardMat.val[1], forwardMat.val[2] };
        cv::Matx31f rightMat = rotationMat.col(0);
        Right = { rightMat.val[0], rightMat.val[1], rightMat.val[2] };
        cv::Matx31f upMat = rotationMat.col(1);
        Up = { upMat.val[0], upMat.val[1], upMat.val[2] };

        // Cached values for performance.
        float distanceAlphaToOmega = FarDepth - NearDepth;
        Centroid = { WorldPosition + Forward * NearDepth * VoiSettings.KernelDepthModifier };
        DistanceAlphaToXi = NearDepth * tan(min(VoiSettings.KernelAngleXRads, VoiSettings.KernelAngleYRads));
        ModifiedDistanceAlphaToOmega = distanceAlphaToOmega * VoiSettings.AwayProminence;
    }

    float VOIKeyframe::TeardropScore(const cv::Vec3f& point) const
    {
        // Alpha is centroid, canonically (barring a shift) the point where the view ray intersects the near plane.
        // Omega is the point where the view ray intersects the far plane.
        // Xi is canonically (barring a shift) either of the points nearest to the view ray where the near plane intersects the view frustum.
        const cv::Vec3f alphaToPoint = point - Centroid;

        const float distancePointToAlpha = sqrt(alphaToPoint.dot(alphaToPoint)); // NearDepth * VoiSettings.KernelDepthModifier

        if (distancePointToAlpha == 0)
        {
            return 1.f;
        }

        const float angleFromForward = acos(alphaToPoint.dot(Forward) / distancePointToAlpha);

        // Compute the modifier for distance.  This gives the kernel its teardrop shape.
        const float parallelBias = 2.f * abs(angleFromForward - mira::HALF_PI<float>) / mira::PI<float>;
        const float directSlope = 1.f / ModifiedDistanceAlphaToOmega + angleFromForward * (1.f / VoiSettings.TowardProminence - 1.f) / (ModifiedDistanceAlphaToOmega * mira::PI<float>);
        const float angleFactor = parallelBias * directSlope + (1 - parallelBias) / (DistanceAlphaToXi * VoiSettings.SideProminence);

        const float x = angleFactor * distancePointToAlpha;
        return pow(mira::E<float>, -2 * x * x);
    }

    bool CalculateVolumeOfInterest(gsl::span<const VOIKeyframe> keyframes, const VolumeOfInterestSettings& voiSettings, AxisAlignedVolume& voi)
    {
        // If there are insufficient keyframes to build a volume of interest, bail out here.
        if (keyframes.size() == 0)
        {
            return false;
        }

        mira::determinator& determinism = mira::determinator::create("VolumeOfInterest");

        SkeletonLogger::PoseHistory::LogVolumeOfInterestArgs<const VOIKeyframe>(keyframes, voiSettings);
        vector<cv::Vec3f> frustumBounds;
        for (const auto& keyframe : keyframes)
        {
            // TODO: Use keyframe-wise kernel angles when available, as opposed to a setting.
            float offsetX = keyframe.NearDepth * tan(voiSettings.KernelAngleXRads / 2);
            float offsetY = keyframe.NearDepth * tan(voiSettings.KernelAngleYRads / 2);

            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.NearDepth + offsetX * keyframe.Right + offsetY * keyframe.Up);
            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.NearDepth + offsetX * keyframe.Right - offsetY * keyframe.Up);
            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.NearDepth - offsetX * keyframe.Right + offsetY * keyframe.Up);
            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.NearDepth - offsetX * keyframe.Right - offsetY * keyframe.Up);

            offsetX *= keyframe.FarDepth / keyframe.NearDepth;
            offsetY *= keyframe.FarDepth / keyframe.NearDepth;

            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.FarDepth + offsetX * keyframe.Right + offsetY * keyframe.Up);
            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.FarDepth + offsetX * keyframe.Right - offsetY * keyframe.Up);
            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.FarDepth - offsetX * keyframe.Right + offsetY * keyframe.Up);
            frustumBounds.emplace_back(keyframe.WorldPosition + keyframe.Forward * keyframe.FarDepth - offsetX * keyframe.Right - offsetY * keyframe.Up);
        }

        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float minZ = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::min();
        float maxY = std::numeric_limits<float>::min();
        float maxZ = std::numeric_limits<float>::min();

        for (const auto& point : frustumBounds)
        {
            minX = min(minX, (point)[0]);
            minY = min(minY, (point)[1]);
            minZ = min(minZ, (point)[2]);

            maxX = max(maxX, (point)[0]);
            maxY = max(maxY, (point)[1]);
            maxZ = max(maxZ, (point)[2]);
        }

        float minVoxelValue = std::numeric_limits<float>::max();
        float maxVoxelValue = std::numeric_limits<float>::min();

        // Pre-compute the scores at all keyframe centroids.  These will be used to
        // help ensure sane bounding boxes.  Use these centroids to initialize the
        // voxel mins and maxes.
        vector<float> keyframeScores;
        keyframeScores.reserve(keyframes.size());
        for (const VOIKeyframe& kf1 : keyframes)
        {
            float pointValue = 0.f;
            for (const VOIKeyframe& kf2 : keyframes)
            {
                pointValue += kf2.TeardropScore(kf1.Centroid);
            }
            keyframeScores.push_back(pointValue);

            minVoxelValue = min(pointValue, minVoxelValue);
            maxVoxelValue = max(pointValue, maxVoxelValue);
        }

        for (int lod = voiSettings.Iterations; lod > 0; lod--)
        {
            cv::Vec3f minPoint{ minX, minY, minZ };

            cv::Vec3f diagonal{ maxX - minX, maxY - minY, maxZ - minZ };

            int voxelCount = voiSettings.VoxelCountFloor / (int)pow(2, lod);
            float voxelSideLength = pow(diagonal[0] * diagonal[1] * diagonal[2] / voxelCount, 1.f / 3.f);
            float voxelHalfSideLength = voxelSideLength / 2.f;

            VoxelSpace voxelSpace(diagonal / voxelSideLength);

            // Compute the teardrop scores of all voxel positions.
            for (int z = 0; z < voxelSpace.dimensions[2]; z++)
            {
                for (int y = 0; y < voxelSpace.dimensions[1]; y++)
                {
                    for (int x = 0; x < voxelSpace.dimensions[0]; x++)
                    {
                        cv::Vec3f point{ minPoint + cv::Vec3f{ x * voxelSideLength, y * voxelSideLength, z * voxelSideLength } };
                        float voxelValue = 0.f;

                        for (const auto& keyframe : keyframes)
                        {
                            voxelValue += keyframe.TeardropScore(point);
                        }

                        voxelSpace.SetVoxel(x, y, z, voxelValue);
                        minVoxelValue = min(minVoxelValue, voxelValue);
                        maxVoxelValue = max(maxVoxelValue, voxelValue);
                    }
                }
            }

            minX = std::numeric_limits<float>::max();
            minY = std::numeric_limits<float>::max();
            minZ = std::numeric_limits<float>::max();
            maxX = std::numeric_limits<float>::min();
            maxY = std::numeric_limits<float>::min();
            maxZ = std::numeric_limits<float>::min();

            float threshold = (maxVoxelValue - minVoxelValue) * (voiSettings.Threshold / lod) + minVoxelValue;

            for (int z = 0; z < voxelSpace.dimensions[2]; z++)
            {
                for (int y = 0; y < voxelSpace.dimensions[1]; y++)
                {
                    for (int x = 0; x < voxelSpace.dimensions[0]; x++)
                    {
                        if (voxelSpace.GetVoxel(x, y, z) > threshold)
                        {
                            cv::Vec3f point{ minPoint + cv::Vec3f{ x * voxelSideLength, y * voxelSideLength, z * voxelSideLength } };

                            minX = min(minX, point[0] - voxelHalfSideLength);
                            minY = min(minY, point[1] - voxelHalfSideLength);
                            minZ = min(minZ, point[2] - voxelHalfSideLength);

                            maxX = max(maxX, point[0] + voxelHalfSideLength);
                            maxY = max(maxY, point[1] + voxelHalfSideLength);
                            maxZ = max(maxZ, point[2] + voxelHalfSideLength);
                        }
                    }
                }
            }

            // Consider the keyrame centroids as well as the voxels.  This eliminates the
            // possibility that an entire keyframe will be eliminated simply because
            // resolution was too low.
            for (int idx = 0; idx < keyframes.size(); idx++)
            {
                if (keyframeScores[idx] > threshold)
                {
                    minX = min(minX, keyframes[idx].Centroid[0] - voxelHalfSideLength);
                    minY = min(minY, keyframes[idx].Centroid[1] - voxelHalfSideLength);
                    minZ = min(minZ, keyframes[idx].Centroid[2] - voxelHalfSideLength);

                    maxX = max(maxX, keyframes[idx].Centroid[0] + voxelHalfSideLength);
                    maxY = max(maxY, keyframes[idx].Centroid[1] + voxelHalfSideLength);
                    maxZ = max(maxZ, keyframes[idx].Centroid[2] + voxelHalfSideLength);
                }
            }
        }

        DETERMINISTIC_CHECK(determinism, minX);
        DETERMINISTIC_CHECK(determinism, minY);
        DETERMINISTIC_CHECK(determinism, minZ);
        DETERMINISTIC_CHECK(determinism, maxX);
        DETERMINISTIC_CHECK(determinism, maxY);
        DETERMINISTIC_CHECK(determinism, maxZ);

        voi.Min.X = minX;
        voi.Min.Y = minY;
        voi.Min.Z = minZ;
        voi.Max.X = maxX;
        voi.Max.Y = maxY;
        voi.Max.Z = maxZ;

        SkeletonLogger::PoseHistory::LogVolumeOfInterest(voi);

        return true;
    }
}
