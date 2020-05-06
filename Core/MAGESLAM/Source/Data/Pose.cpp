// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// Pose.h
//
// Stores the camera pose as a 3d position and rotation.
// it is the transform that will bring things into a camera relative space, not 
// the transform that would position a proxy object into the orientation of the
// camera
//------------------------------------------------------------------------------

#include "Pose.h"

#include "Utils\cv.h"

#include <Eigen\Dense>
#include <Eigen\Geometry> 
#include <opencv2\core\eigen.hpp>
#include <opencv2\calib3d\calib3d.hpp>

namespace mage
{
    Pose::Pose()
    {
        // initialization 
        m_viewMatrix = cv::Matx34f::eye();
        m_inverseViewMatrix = cv::Matx44f::eye();
    }

    Pose::Pose(const cv::Matx44f& worldMatrix)
    {
        m_inverseViewMatrix = worldMatrix;
        cv::Matx44f temp = Invert(worldMatrix);
        m_viewMatrix = temp.get_minor<3, 4>(0, 0);
    }

    Pose::Pose(const cv::Matx31f& viewSpacePosition, const cv::Matx33f& viewSpaceRotationMatrix)
    {
        SetViewSpacePositionAndRotation(viewSpacePosition, viewSpaceRotationMatrix);
    }

    void Pose::SetViewMatrix(const cv::Matx34f& viewMat)
    {
            m_viewMatrix(0, 0) = viewMat(0, 0);
            m_viewMatrix(0, 1) = viewMat(0, 1);
            m_viewMatrix(0, 2) = viewMat(0, 2);
            m_viewMatrix(0, 3) = viewMat(0, 3);

            m_viewMatrix(1, 0) = viewMat(1, 0);
            m_viewMatrix(1, 1) = viewMat(1, 1);
            m_viewMatrix(1, 2) = viewMat(1, 2);
            m_viewMatrix(1, 3) = viewMat(1, 3);

            m_viewMatrix(2, 0) = viewMat(2, 0);
            m_viewMatrix(2, 1) = viewMat(2, 1);
            m_viewMatrix(2, 2) = viewMat(2, 2);
            m_viewMatrix(2, 3) = viewMat(2, 3);

            ComputeInverseViewMatrix();
    }

    void Pose::SetViewSpacePositionAndRotation(const cv::Matx31f& positionMatrix, const cv::Matx33f& viewSpaceRotationMatrix)
    {
        m_viewMatrix(0, 3) = positionMatrix(0,0);
        m_viewMatrix(1, 3) = positionMatrix(1,0);
        m_viewMatrix(2, 3) = positionMatrix(2,0);

        m_viewMatrix(0, 0) = viewSpaceRotationMatrix(0, 0);
        m_viewMatrix(0, 1) = viewSpaceRotationMatrix(0, 1);
        m_viewMatrix(0, 2) = viewSpaceRotationMatrix(0, 2);

        m_viewMatrix(1, 0) = viewSpaceRotationMatrix(1, 0);
        m_viewMatrix(1, 1) = viewSpaceRotationMatrix(1, 1);
        m_viewMatrix(1, 2) = viewSpaceRotationMatrix(1, 2);

        m_viewMatrix(2, 0) = viewSpaceRotationMatrix(2, 0);
        m_viewMatrix(2, 1) = viewSpaceRotationMatrix(2, 1);
        m_viewMatrix(2, 2) = viewSpaceRotationMatrix(2, 2);

        ComputeInverseViewMatrix();
    }

    cv::Matx31f Pose::GetViewSpacePosition() const
    {
        return m_viewMatrix.col(3);
    }

    cv::Matx31f Pose::GetViewSpaceRodriguesRotation() const
    {
        cv::Matx31f rotation;
        cv::Rodrigues(GetRotationMatrix(), rotation);
        return rotation;
    }

    float Pose::GetRoll() const
    {
        // Todo: perf. converting from mat to quat back to mat to calculate roll.
        auto quat = ToQuat(Rotation(m_inverseViewMatrix));

        float yaw;
        float pitch;
        float roll;

        ToEuler(quat, yaw, pitch, roll);

        return roll;
    }

    cv::Point3f Pose::GetWorldSpacePosition() const
    {
        return{ m_inverseViewMatrix(0,3), m_inverseViewMatrix(1,3), m_inverseViewMatrix(2,3) };
    }

    cv::Vec3f Pose::GetWorldSpaceForward() const
    {
        return{ m_inverseViewMatrix(0,2), m_inverseViewMatrix(1,2), m_inverseViewMatrix(2,2) };
    }

    cv::Vec3f Pose::GetWorldSpaceRight() const
    {
        return{ m_inverseViewMatrix(0,0), m_inverseViewMatrix(1,0), m_inverseViewMatrix(2,0) };
    }

    // right handed, column major
    cv::Matx33f Pose::GetRotationMatrix() const
    {
        return m_viewMatrix.get_minor<3, 3>(0, 0);
    }

    // use eigen to handle the rotation matrix <=> quaternion conversion
    Quaternion Pose::GetRotationQuaternion() const
    {
        Eigen::Matrix3f eigenMat;
        cv::cv2eigen(GetRotationMatrix(), eigenMat);

        return Eigen::Quaternionf{ eigenMat }.normalized();
    }

    //right handed, column major
    const cv::Matx34f& Pose::GetViewMatrix() const
    {
        return m_viewMatrix;
    }

    cv::Matx44f Pose::GetViewMatrix4x4() const
    {
        return {
            m_viewMatrix(0, 0), m_viewMatrix(0, 1), m_viewMatrix(0, 2), m_viewMatrix(0, 3),
            m_viewMatrix(1, 0), m_viewMatrix(1, 1), m_viewMatrix(1, 2), m_viewMatrix(1, 3),
            m_viewMatrix(2, 0), m_viewMatrix(2, 1), m_viewMatrix(2, 2), m_viewMatrix(2, 3),
                             0,                  0,                  0,                  1
        };
    }

    //right handed, column major
    const cv::Matx44f& Pose::GetInverseViewMatrix() const
    {
        return m_inverseViewMatrix;
    }

    cv::Matx34f Pose::GetRelativeViewMatrix(const Pose& toFrame) const
    {
        return toFrame.GetViewMatrix() * m_inverseViewMatrix;
    }

    void Pose::ComputeInverseViewMatrix()
    {
        m_inverseViewMatrix = Invert(m_viewMatrix);
    }
}