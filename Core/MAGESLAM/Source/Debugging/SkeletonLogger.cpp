// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "SkeletonLogger.h"

#include <sstream>

// TODO: Tracing

#include "Data/Pose.h"
#include "Data/Types.h"
#include "Debugging/SkeletonKey.h"
#include "Map/CovisibilityGraph.h"
#include "Utils/cv.h"

using namespace std;

namespace mage
{
    SkeletonLoggerLevel SkeletonLogger::s_level = SkeletonLoggerLevel::Off;

    // Every message may have overhead data while being sent, so that the message itself can not be 65536 
    // this number is got from experiments  
    const size_t ETW_MESSAGE_SIZE_LIMIT = 65000;
    
    void SkeletonLogger::SetLevel(SkeletonLoggerLevel newLevel)
    {
        s_level = newLevel;
    }

    bool SkeletonLogger::LoggingEnabled(SkeletonLoggerLevel level)
    {
        unsigned int query = (unsigned int)level;
        unsigned int metric = (unsigned int)s_level;
        return (query & metric) != 0 && (query & ~metric) == 0;
    }

    void SkeletonLogger::Log(const char *data)
    {
        // TODO: Write the data to some logging handler.
    }

    void SkeletonLogger::Log(const void *data, size_t length)
    {
        // TODO: Write the data to some logging handler.
    }

    // ---------- TRACKING THREAD ----------
    void SkeletonLogger::TrackingThread::LogBoundingPlaneDepths(const KeyframeProxy& keyframe, const Depth& depth)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Tracking))
            return;

        stringstream ss;
        ss << "depths," << reinterpret_cast<const IdT&>(keyframe.GetId()) << "," << depth.NearPlaneDepth << "," << depth.FarPlaneDepth << ",";
        Log(ss.str().c_str());
    }

    // ---------- TRACK LOCAL MAP ----------
    void SkeletonLogger::TrackLocalMap::LogPose(unsigned int ordinal, const KeyframeProxy& keyframe, const Pose& pose)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Tracking))
            return;

        stringstream ss;
        cv::Point3f p = pose.GetWorldSpacePosition();
        cv::Vec3f f = pose.GetWorldSpaceForward();
        cv::Vec3f r = pose.GetWorldSpaceRight();
        ss << "pose" << ordinal << "," << reinterpret_cast<const IdT&>(keyframe.GetId()) << ","
            << p.x << "," << p.y << "," << p.z << ","
            << f[0] << "," << f[1] << "," << f[2] << ","
            << r[0] << "," << r[1] << "," << r[2] << ",";
        Log(ss.str().c_str());
    }

    void SkeletonLogger::TrackLocalMap::LogAssociatedCount(unsigned int ordinal, const KeyframeProxy & keyframe)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Tracking))
            return;

        stringstream ss;
        ss << "numassociated" << ordinal << "," << keyframe.GetAssociatedKeypointCount();
        Log(ss.str().c_str());
    }

    void SkeletonLogger::TrackLocalMap::LogVisitedMapPoints(const mage::temp::unordered_set<Id<MapPoint>>& mapPointIds)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Tracking))
            return;

        stringstream idStream;
        idStream << "visited";

        for (const Id<MapPoint>& id : mapPointIds)
        {
            idStream << "," << id;
        }

        Log(idStream.str().c_str());
    }

    void SkeletonLogger::TrackLocalMap::LogKeypoints(const KeyframeProxy & keyframe)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Tracking))
            return;

        stringstream unass;
        unass << "unassociated,";
        stringstream assoc;
        assoc << "associated,";

        auto points = keyframe.GetAnalyzedImage()->GetKeyPoints();
        auto mask = keyframe.GetAssociatedKeypointMask();

        for (unsigned int idx = 0; idx < mask.size(); idx++)
        {
            const cv::KeyPoint& point = points[idx];
            if (mask[idx])
            {
                // Log associated keypoint.
                const auto& assocMp = keyframe.GetAssociatedMapPoint(idx);
                const auto& assocPos = assocMp->GetPosition();
                assoc << reinterpret_cast<const IdT&>(assocMp->GetId()) << "," << assocPos.x << "," << assocPos.y << "," << assocPos.z << "," << point.pt.x << "," << point.pt.y << ",";
            }
            else
            {
                // Log unassociated keypoint.
                unass << point.pt.x << "," << point.pt.y << ",";
            }
        }

        Log(unass.str().c_str());
        Log(assoc.str().c_str());
    }

    // ---------- IMAGE THREAD ----------
    void SkeletonLogger::ImageLogging::LogImage(const KeyframeProxy& /*keyframe*/)
    {
        return; //TODO: needs the cv::mat conditionally compiled into FrameData on the tracking thread

        /*if (!LoggingEnabled(SkeletonLoggerLevel::Image))
            return;

        auto image = keyframe.GetAnalyzedImage()->GetImageData()->GetImageMatrix();
        const void *data = image.data;

        int width = image.size().width;
        int height = image.size().height;

        stringstream ss;
        ss << "image," << reinterpret_cast<const IdT&>(keyframe.GetId()) << "," << width << "," << height;
        ss << "," << keyframe.GetUndistortedIntrinsics().GetCx();
        ss << "," << keyframe.GetUndistortedIntrinsics().GetCy();
        ss << "," << keyframe.GetUndistortedIntrinsics().GetFx();
        ss << "," << keyframe.GetUndistortedIntrinsics().GetFy();
        Log(ss.str().c_str());

        Log(data, width * height);*/
    }

    void SkeletonLogger::LogBigBuffer(gsl::span<const uint8_t> buffer, const char *identifier)
    {
        std::stringstream ss;
        ss << identifier << "," ;
        std::string prefix = ss.str();

        assert(prefix.length() < ETW_MESSAGE_SIZE_LIMIT && "The identifier's size is too big to put in a single message. Please rename it.");

        uint8_t sendBuffer[ETW_MESSAGE_SIZE_LIMIT];
        memcpy(sendBuffer, prefix.data(), prefix.length());

        // Send messages containing actual data
        size_t bufLen = buffer.size_bytes();
        for (size_t pos = 0; pos < bufLen; )
        {
            size_t dataLen = std::min(bufLen - pos, ETW_MESSAGE_SIZE_LIMIT - prefix.length());
            memcpy(sendBuffer + prefix.length(), &buffer[pos], dataLen);
           
            // Prevent the EtwClient in the magedebugger tool overwhelmed by messages in a short time
            // TOOD: Tracing Sleep(1);
                        
            Log(sendBuffer, prefix.length() + dataLen);
            pos += dataLen;
        }
        // Send final message containing no data, only the ID, to indicate that transmission is complete.
        Log(sendBuffer, prefix.length());    
    }

    /*----------------------Mesh Vertices-----------------------------------*/
    void SkeletonLogger::Mesh::LogVertices(gsl::span<const VNormalTexture> vdata)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Model))
            return;

        std::stringstream ss;
        ss << "vertices,";
        for (auto data : vdata)
        {
           ss << data.pos.x << "," << data.pos.y << "," << data.pos.z << "," << data.norm.x << "," << data.norm.y << "," << data.norm.z\
              << "," << data.tex.x << "," << data.tex.y << ",";
           if (ss.tellp() > gsl::narrow<std::iostream::pos_type>(ETW_MESSAGE_SIZE_LIMIT))
           {
               Log(ss.str().c_str());
               ss.str("");
               ss << "vertices,";
           }
        }
        Log(ss.str().c_str());
    }

    /*----------------------Mesh Indices-----------------------------------*/
    void SkeletonLogger::Mesh:: LogIndices(gsl::span<const uint32_t> idxData)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Model))
            return;

        std::stringstream ss;
        ss << "indices,";
        for (uint32_t index : idxData)
        {
            ss << index << ",";
            if (ss.tellp() > gsl::narrow<std::iostream::pos_type>(ETW_MESSAGE_SIZE_LIMIT))
            {
                Log(ss.str().c_str());
                ss.str("");
                ss << "indices,";
            }
        }
        Log(ss.str().c_str());
    }
    
    /*----------------------Mesh Texture-----------------------------------*/
    void SkeletonLogger::Mesh::LogTexture(size_t width, size_t height, gsl::span<const uint8_t> pixels)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Model))
            return;
        
        std::stringstream ss;
        ss << "texture,";
        ss << width << ","  << height << "," << pixels.size_bytes() << ",";
        Log(ss.str().c_str());
        
        LogBigBuffer(pixels, "texture");
    }

    /*----------------------Updated Pose afte Global BA-----------------------------------*/
    void SkeletonLogger::UpdatedPose::LogUpdatedPose(const mage::PoseHistory& history)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::GloballyAdjustedPoses))
            return;

        std::vector<Pose> poses;
        history.DebugGetAllPoses(poses);
        auto& m_historicalPoses = history.GetHistoricalPoses();

        stringstream ss;
        ss << "updatedPoses,";

        for (size_t i = 0; i < poses.size(); i++)
        {
            cv::Point3f p = poses[i].GetWorldSpacePosition();
            cv::Vec3f f = poses[i].GetWorldSpaceForward();
            cv::Vec3f r = poses[i].GetWorldSpaceRight();
            ss << m_historicalPoses[i].GetId() << ","
               << p.x << "," << p.y << "," << p.z << ","
               << f[0] << "," << f[1] << "," << f[2] << ","
               << r[0] << "," << r[1] << "," << r[2] << ",";
            if (ss.tellp() > gsl::narrow<std::iostream::pos_type>(ETW_MESSAGE_SIZE_LIMIT))
            {
                Log(ss.str().c_str());
                ss.str("");
                ss << "updatedPoses,";
            }
        }
        Log(ss.str().c_str());
        Log("updatedPoses Done");
    }

    // ---------- MAP INITIALIZATION ----------
    void SkeletonLogger::MapInitialization::LogMatches(gsl::span<const cv::DMatch> matches, const AnalyzedImage& queryFrame, const AnalyzedImage& trainFrame)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Initialization))
            return;

        stringstream ss;
        ss << "initpoints,";
        for (ptrdiff_t idx = 0; idx < matches.size(); idx++)
        {
            const cv::KeyPoint& queryMatch = queryFrame.GetKeyPoint(matches[idx].queryIdx);
            const cv::KeyPoint& trainMatch = trainFrame.GetKeyPoint(matches[idx].trainIdx);
            ss << queryMatch.pt.x << "," << queryMatch.pt.y << "," << trainMatch.pt.x << "," << trainMatch.pt.y << ",";
        }
        Log(ss.str().c_str());
    }

    void SkeletonLogger::MapInitialization::LogFailure(const char *message)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Initialization))
            return;

        stringstream ss;
        ss << "initfailed," << message;
        Log(ss.str().c_str());
    }

    // ---------- MAP ----------
    void SkeletonLogger::Map::LogMapPointsSnaphot(const unordered_map<Id<MapPoint>, unique_ptr<MapPoint>>& map_points)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Mapping))
            return;

        stringstream ss;
        ss << "mappoints,";
        for (auto itr = map_points.begin(); itr != map_points.end(); itr++)
        {
            Id<MapPoint> id = itr->first;
            const unique_ptr<MapPoint>& mp = itr->second;

            ss << reinterpret_cast<const IdT&>(id) << "," << mp->GetPosition().x << "," << mp->GetPosition().y << "," << mp->GetPosition().z << ",";
        }
        Log(ss.str().c_str());
    }

    void SkeletonLogger::Map::LogKeyframesSnapshot(gsl::span<const std::unique_ptr<Keyframe>> keyframes, const CovisibilityGraph& covisGraph)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Mapping))
            return;

        stringstream ss;
        ss << "keyframes,";
        
        for (ptrdiff_t idx = 0; idx < keyframes.size(); idx++)
        {
            const Keyframe& keyframe = *keyframes[idx];
            Id<Keyframe> id = keyframe.GetId();
            cv::Point3f p = keyframe.GetPose().GetWorldSpacePosition();
            cv::Vec3f f = keyframe.GetPose().GetWorldSpaceForward();
            cv::Vec3f r = keyframe.GetPose().GetWorldSpaceRight();
            ss << reinterpret_cast<const IdT&>(id) << "," << p.x << "," << p.y << "," << p.z << "," << f[0] << "," << f[1] << "," << f[2] << "," << r[0] << "," << r[1] << "," << r[2] << ",";

            for (int kfIdx = 0; kfIdx < idx; kfIdx++)
            {
                ss << covisGraph.GetNumSharedMapPoints(keyframes[kfIdx]->GetId(), keyframes[idx]->GetId()) << ",";
            }
        }

        Log(ss.str().c_str());
    }

    void SkeletonLogger::PoseHistory::LogConfidenceSpace(float *space, size_t dimX, size_t dimY, size_t dimZ)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Mapping))
            return;

        size_t totalSize = sizeof(float) * dimX * dimY * dimZ;
        stringstream identifier;
        identifier << "confidencespace," << dimX << "," << dimY << "," << dimZ;
        LogBigBuffer({(const uint8_t *)space, (int)totalSize}, identifier.str().c_str());
    }

    void SkeletonLogger::PoseHistory::LogVolumeOfInterest(const AxisAlignedVolume& voi)
    {
        if (!LoggingEnabled(SkeletonLoggerLevel::Mapping))
            return;

        stringstream ss;
        ss << "volumeofinterest," <<
            voi.Min.X << "," <<
            voi.Min.Y << "," <<
            voi.Min.Z << "," <<
            voi.Max.X << "," <<
            voi.Max.Y << "," <<
            voi.Max.Z << ",";

        Log(ss.str().c_str());
    }
}
