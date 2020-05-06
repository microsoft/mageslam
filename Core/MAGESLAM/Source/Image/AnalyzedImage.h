// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Device/CameraCalibration.h"
#include "OrbFeatureDetector.h"
#include "KeypointSpatialIndex.h"
#include "ImageData.h"

#include "Data/Pose.h"

#include <arcana/utils/serialization/serializable.h>
#include <arcana/analysis/introspector.h>

#include "opencv2/core/core.hpp"

#include <memory>
#include <chrono>

namespace mage
{
//    #define DBG_KEEPIMAGES //uncomment this line to keep images in the frame data for debugging

    class AnalyzedImage : public mira::serializable<AnalyzedImage>
    {
    public:
        using time_stamp = std::chrono::system_clock::time_point;

        AnalyzedImage(ConstImageHandle image, const FrameId& frameId,
                const CameraCalibration& distortedCalibration, const CameraCalibration& undistortedCalibration, const time_stamp& timestamp, const cv::Mat& debugImage)
            :   m_image{ move(image) },
                m_frameId{ frameId },
                m_distortedCalibration{ distortedCalibration },
                m_undistortedCalibration{ undistortedCalibration },
                m_spatialIndex(m_image->GetKeypoints()),
                m_timestamp{ timestamp },
#ifdef DBG_KEEPIMAGES
                m_debugImage{ debugImage }
#else
                m_debugImage{}
#endif
        {
            assert(m_frameId.Camera == m_image->GetCameraIdentity() && "mismatched frame id camera type");

#ifndef DBG_KEEPIMAGES
            UNUSED(debugImage);
#endif
        }

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        AnalyzedImage(ConstImageHandle image, StreamT& stream)
            :   m_image{ move(image) },
                m_frameId{ 0, mage::CameraIdentity::MONO },
                m_distortedCalibration{ },
                m_undistortedCalibration{ },
                m_spatialIndex{ m_image->GetKeypoints() },
                m_timestamp{ }
        {
            deserialize(stream);
        }

        static constexpr auto members()
        {
            return declare_members(
                &AnalyzedImage::m_frameId,
                &AnalyzedImage::m_distortedCalibration,
                &AnalyzedImage::m_undistortedCalibration,
                &AnalyzedImage::m_timestamp
            );
        }

        const size_t GetDescriptorsCount() const
        {
            return m_image->GetFeatureCount();
        }

        gsl::span<const ORBDescriptor> GetDescriptors() const
        {
            return m_image->GetDescriptors();
        }

        const ORBDescriptor& GetDescriptor(size_t index) const
        {
            return m_image->GetDescriptor(index);
        }

        gsl::span<const cv::KeyPoint> GetKeyPoints() const
        {
            return m_image->GetKeypoints();
        }
     
        const KeypointSpatialIndex& GetKeypointSpatialIndex() const
        {
            return m_spatialIndex;
        }

        const cv::KeyPoint& GetKeyPoint(size_t index) const
        {
            return m_image->GetKeypoint(index);
        }
       
        //pixels
        uint32_t GetWidth() const
        {
            return m_undistortedCalibration.GetCalibrationWidth();
        }

        //pixels
        uint32_t GetHeight() const
        {
            return m_undistortedCalibration.GetCalibrationHeight();
        }

        // Method is similar in spirit to 'contains' but takes into account a user
        // specified border around the image.  Negative border values have the effect
        // of increasing the image size for the purposes of this comparison
        bool PointWithinImageBorder(const cv::Point2f& point, float imageBorder) const
        {
            return imageBorder <= point.x && imageBorder <= point.y &&
                point.x < GetWidth() - imageBorder && point.y < GetHeight() - imageBorder;
        }

        bool Contains(const cv::Point2f& point) const
        {
            return 0 <= point.x && 0 <= point.y &&
                point.x < GetWidth() && point.y < GetHeight();
        }

        const CameraCalibration& GetDistortedCalibration() const  //TODO: remove distorted calibration from analyzed image (remove from pose history)
        {
            return m_distortedCalibration;
        }
        const CameraCalibration& GetUndistortedCalibration() const
        {
            return m_undistortedCalibration;
        }

        float GetPyramidScale() const
        {
            return m_image->GetPyramidScale();
        }

        size_t GetNumLevels() const
        {
            return m_image->GetNumLevels();
        }

        float GetImageBorder() const
        {
            return m_image->GetImageBorder();
        }

        CameraIdentity GetCameraIdentity() const
        {
            return m_image->GetCameraIdentity();
        }

        size_t GetMaxFeatures() const
        {
            return m_image->GetMaxFeatures();
        }

        void MarkAsPublished() const
        {
            m_image->MarkAsPublished();
        }

        const time_stamp& GetTimeStamp() const
        {
            return m_timestamp;
        }

        const ConstImageHandle& GetImageData() const
        {
            return m_image;
        }

        const FrameId& GetFrameId() const
        {
            return m_frameId;
        }

        const cv::Mat& GetDebugImage() const
        {
            return m_debugImage;
        }

    private:
        const ConstImageHandle m_image;
        const FrameId m_frameId;
        const CameraCalibration m_distortedCalibration;
        const CameraCalibration m_undistortedCalibration;
        KeypointSpatialIndex m_spatialIndex;
        const time_stamp m_timestamp;
        const cv::Mat m_debugImage;
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const FrameId& object)
    {
        intro(
            cereal::make_nvp("CorrelationId", object.CorrelationId),
            cereal::make_nvp("Camera", object.Camera)
        );
    }

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const AnalyzedImage& image)
    {
        intro(
            cereal::make_nvp("FrameId", image.GetFrameId()),
            cereal::make_nvp("TimeStamp", image.GetTimeStamp().time_since_epoch().count()),
            cereal::make_nvp("FeatureCount", image.GetImageData()->GetFeatureCount())
        );
    }
}
