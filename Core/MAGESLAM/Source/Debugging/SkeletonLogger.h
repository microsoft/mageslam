// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <memory>
#include "opencv\cv.h"
#include "Containers\GenericContainers.h"
#include "Data\Types.h"

#include "Proxies\KeyframeProxy.h"
#include "Debugging\SkeletonKey.h"
#include "Map\PoseHistory.h"

namespace mage
{
    // Forward declares
    class AnalyzedImage;
    class CovisibilityGraph;
    struct Depth;
    class Keyframe;
    class MapPoint;
    class Pose;
    enum class SkeletonLoggerLevel;

    class SkeletonLogger
    {
    public:
        static void SetLevel(SkeletonLoggerLevel);

        class TrackingThread
        {
        public:
            static void LogBoundingPlaneDepths(const KeyframeProxy& keyframe, const Depth& depth);
        };

        class TrackLocalMap
        {
        public:
            static void LogPose(unsigned int ordinal, const KeyframeProxy& keyframe, const Pose& pose);
            static void LogAssociatedCount(unsigned int ordinal, const KeyframeProxy& keyframe);
            static void LogKeypoints(const KeyframeProxy& keyframe);
            static void LogVisitedMapPoints(const mage::temp::unordered_set<Id<MapPoint>>& mapPointIds);
        };

        class ImageLogging
        {
        public:
            static void LogImage(const KeyframeProxy& keyframe);
        };

        class MapInitialization
        {
        public:
            static void LogMatches(gsl::span<const cv::DMatch> matches, const AnalyzedImage& queryFrame, const AnalyzedImage& trainFrame);
            static void LogFailure(const char *message);
        };

        class Map
        {
        public:
            static void LogMapPointsSnaphot(const std::unordered_map<Id<MapPoint>, std::unique_ptr<MapPoint>>& map_points);
            static void LogKeyframesSnapshot(gsl::span<const std::unique_ptr<Keyframe>> keyframes, const CovisibilityGraph&);
        };

        class Mesh
        {
        public:
            struct FLOAT3
            {
                float x;
                float y;
                float z;
            };
            struct FLOAT2
            {
                float x;
                float y;
            };
            struct VNormalTexture
            {
                FLOAT3 pos;
                FLOAT3 norm;
                FLOAT2 tex;
            };

            static void LogVertices(gsl::span<const VNormalTexture> vdata);
            static void LogIndices(gsl::span<const uint32_t> idxPtr);
            static void LogTexture(size_t width, size_t height, gsl::span<const uint8_t> pixels);           
        };

        class UpdatedPose
        {
        public:
            static void LogUpdatedPose(const mage::PoseHistory& history);
        };

        class PoseHistory
        {
        public:
            template<class TKeyframe>
            static void LogVolumeOfInterestArgs(gsl::span<TKeyframe> keyframes, const VolumeOfInterestSettings& settings)
            {
                if (!LoggingEnabled(SkeletonLoggerLevel::Mapping))
                    return;

                stringstream ss;
                ss << "volumeofinterestargs,";

                ss << settings.Threshold << "," <<
                    settings.Iterations << "," <<
                    settings.VoxelCountFloor << "," <<
                    settings.AwayProminence << "," <<
                    settings.TowardProminence << "," <<
                    settings.SideProminence << "," <<
                    settings.KernelAngleXRads << "," <<
                    settings.KernelAngleYRads << "," <<
                    settings.KernelPitchRads << "," <<
                    settings.KernelRollRads << "," <<
                    settings.KernelYawRads << "," <<
                    settings.KernelDepthModifier << ",";

                for (const auto& kf : keyframes)
                {
                    ss << kf.WorldPosition << "," <<
                        kf.Forward << "," << kf.Right << "," << kf.Up << "," <<
                        kf.NearDepth << "," << kf.FarDepth << ",";
                }

                Log(ss.str().c_str());
            }

            static void LogConfidenceSpace(float *space, size_t dimX, size_t dimY, size_t dimZ);

            static void LogVolumeOfInterest(const AxisAlignedVolume& voi);
        };
        
    private:
        static SkeletonLoggerLevel s_level;

        // Returns true when logging is enabled for the provided level, false otherwise.
        // Note that this result is trivially false when the provided level is ::Off == 0b0.
        static bool LoggingEnabled(SkeletonLoggerLevel level);

        static void Log(const char *data);
        static void Log(const void *data, size_t length);
        static void LogBigBuffer(gsl::span<const uint8_t>buffer, const char *identifier);
    };
}
