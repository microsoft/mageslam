// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// Pose.h
//
// Stores the camera pose as a 3d position and quaternion rotation.
//------------------------------------------------------------------------------

#pragma once

#include <arcana/utils/serialization/serializable.h>
#include <arcana/analysis/introspector.h>

#include <opencv2/core/core.hpp>
#include "Utils/cv.h"

namespace mage
{
    class Pose : public mira::serializable<Pose>
    {
    public:
        Pose();
        Pose(const cv::Matx44f& worldMatrix);
        Pose(const cv::Matx31f& viewSpacePosition, const cv::Matx33f& viewSpaceRotationMatrix);

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        explicit Pose(StreamT& stream)
        {
            deserialize(stream);
            ComputeInverseViewMatrix();
        }

        static constexpr auto members()
        {
            return declare_members(
                &Pose::m_viewMatrix
            );
        }

        void SetViewMatrix(const cv::Matx34f& viewMat);
        void SetViewSpacePositionAndRotation(const cv::Matx31f& positionMatrix, const cv::Matx33f& rotationMatrix);
        
        cv::Matx31f GetViewSpacePosition() const;
        cv::Matx31f GetViewSpaceRodriguesRotation() const;
        
        cv::Vec3f GetWorldSpaceForward() const;
        cv::Vec3f GetWorldSpaceRight() const;
        cv::Point3f GetWorldSpacePosition() const;

        float GetRoll() const;

        cv::Matx33f GetRotationMatrix() const;
        Quaternion GetRotationQuaternion() const;

        const cv::Matx34f& GetViewMatrix() const;
        cv::Matx44f GetViewMatrix4x4() const;
        const cv::Matx44f& GetInverseViewMatrix() const;

        cv::Matx34f GetRelativeViewMatrix(const Pose& toFrame) const;

    private:
        cv::Matx34f m_viewMatrix;
        cv::Matx44f m_inverseViewMatrix;

        void ComputeInverseViewMatrix();
    };

    template<typename ArchiveT>
    void introspect_object(mira::introspector<ArchiveT>& intro, const Pose& object)
    {
        intro(
            cereal::make_nvp("ViewSpacePosition", gsl::make_span(object.GetViewSpacePosition().val)),
            cereal::make_nvp("ViewSpaceRodriguesRotation", gsl::make_span(object.GetViewSpaceRodriguesRotation().val))
        );
    }
}
