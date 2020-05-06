// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Reprojection.h"
#include "Device/CameraCalibration.h"

#include <opencv2\core.hpp>
#include <opencv2\calib3d.hpp>

namespace mage
{
    /*
    Call this with a vector of 3D points, a camera pose, and a calibration matrix.
    The 3D points will be reprojected into 2D and returned in the points2D vector.
    */
    void ProjectPoints(const gsl::span<const cv::Point3f>& points3D, const cv::Matx34f& cameraPose, const cv::Matx33f& calibrationMatrix, std::vector<Projection>& points2D)
    {
        points2D.reserve(points3D.size());

        for (const auto& point3D : points3D)
        {
            points2D.emplace_back(ProjectUndistorted(cameraPose, calibrationMatrix, point3D));
        }
    }

    Projection ProjectUndistorted(const cv::Matx34f& viewMatrix, const cv::Matx33f& calibration, const cv::Point3f& world)
    {
        cv::Matx31f camSpace = viewMatrix * cv::Matx41f{ world.x, world.y, world.z, 1 };
        float depth = camSpace(2);

        // prevent division by zero
        float divDepth = depth != 0 ? depth : 1;

        // The Calibration matrix is known to be sparse, so we can do this more efficiently by accessing the members directly
        return{
            {
                (camSpace(0) / divDepth) * calibration(0,0) + calibration(0,2),
                (camSpace(1) / divDepth) * calibration(1,1) + calibration(1,2)
            },
            depth
        };
    }

    //TODO: extend to rational6k by calling opencv project function task: 13498830
    Projection ProjectDistorted(
        const CameraCalibration& cameraCalibration,
        const cv::Matx34f& viewMatrix,
        const cv::Point3f& world)
    {
        assert(cameraCalibration.GetDistortionType() == calibration::DistortionType::Poly3k && "projectdistorted only supports poly3k");
        
        cv::Matx31f camSpace = viewMatrix * cv::Matx41f{ world.x, world.y, world.z, 1 };
        float depth = camSpace(2);
        
        //TODO: handle zero depth gracefully        
        cv::Matx31f normalizedCamSpace = cv::Matx31f{ camSpace(0) / depth, camSpace(1) / depth, 1.f };
        float r2 = normalizedCamSpace(0)*  normalizedCamSpace(0) + normalizedCamSpace(1) * normalizedCamSpace(1);
        float r4 = r2 * r2;
        float r6 = r4 * r2;

        // mimics opencv's distortion model
        // k1, k2, p1, p2, k3        
        //http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        cv::Matx31f distortedNormalizedCamSpace;     
        float xnyn = normalizedCamSpace(0) *  normalizedCamSpace(1);
        distortedNormalizedCamSpace(0) = normalizedCamSpace(0) * (1 + cameraCalibration.GetK1()*r2 + cameraCalibration.GetK2()*r4 + cameraCalibration.GetK3()*r6) + 2 * cameraCalibration.GetP1() * xnyn + cameraCalibration.GetP2()*(r2 + 2 * normalizedCamSpace(0)*normalizedCamSpace(0));
        distortedNormalizedCamSpace(1) = normalizedCamSpace(1) * (1 + cameraCalibration.GetK1()*r2 + cameraCalibration.GetK2()*r4 + cameraCalibration.GetK3()*r6) + cameraCalibration.GetP1()*(r2 + 2 * normalizedCamSpace(1) *  normalizedCamSpace(1)) + 2 * cameraCalibration.GetP2() * xnyn;
        distortedNormalizedCamSpace(2) = 1.0f;

        cv::Matx31f distortedScreenSpace = cameraCalibration.GetCameraMatrix() * distortedNormalizedCamSpace;
        
        return{ { distortedScreenSpace(0), distortedScreenSpace(1) }, depth };
    }

    std::vector<cv::Point2f> ProjectDistorted(
        const mage::CameraCalibration& calibration,
        const cv::Matx34f& viewMatrix,
        gsl::span<const cv::Vec3f> points3D)
    {
        assert(calibration.GetDistortionType() != calibration::DistortionType::None && "projectdistorted only for distorted calibrations");

        std::vector<cv::Point2f> results;

        const cv::Mat objectPoints(cv::Size(3, gsl::narrow_cast<int>(points3D.size())), CV_32F, const_cast<void*>(static_cast<const void*>(points3D.data())));

        cv::Vec3f rotation;
        cv::Rodrigues(viewMatrix.get_minor<3, 3>(0, 0), rotation);

        auto translation = viewMatrix.col(3);

        auto camMatrix = calibration.GetCameraMatrix();

        cv::projectPoints(objectPoints, rotation, translation, camMatrix, calibration.GetCVDistortionCoeffs(), results);

        return results;
    }
}
