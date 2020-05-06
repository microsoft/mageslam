// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Triangulation.h"

#include <opencv2\core.hpp>
#include "Utils\cv.h"

namespace mage
{
    void TriangulatePointsViewSpaceSlow(const cv::Matx34f& frame1CameraProjection, const cv::Matx34f& frame2CameraProjection, const std::vector<cv::Point2f>& frame1Matched2dPoints,
        const std::vector<cv::Point2f>& frame2Matched2dPoints, std::vector<cv::Point3f>& triangulated3dPoints)
    {
        cv::Mat points4D;
        cv::triangulatePoints(frame1CameraProjection, frame2CameraProjection, frame1Matched2dPoints, frame2Matched2dPoints, points4D);

        for (int j = 0; j < points4D.cols; j++) {
            cv::Vec4f vec = points4D.col(j);
            vec /= vec(3);
            triangulated3dPoints.push_back({ vec(0), vec(1), vec(2) });
        }
    }

    cv::Point3f TriangulatePointWorldSpace(const cv::Matx33f& invCalibration1, const cv::Matx44f& worldMat1, const cv::Matx33f& invCalibration2, const cv::Matx44f& worldMat2, const cv::Point2f& p1, const cv::Point2f& p2)
    {
        cv::Vec3f pointcamera1 = invCalibration1 * cv::Vec3f{ p1.x, p1.y, 1.0f };
        cv::Vec3f pointcamera2 = invCalibration2 * cv::Vec3f{ p2.x, p2.y, 1.0f };

        cv::Vec4f worldRay1 = worldMat1 * cv::Vec4f{ pointcamera1(0),pointcamera1(1),pointcamera1(2), 1.0f };
        cv::Vec4f worldRay2 = worldMat2 * cv::Vec4f{ pointcamera2(0),pointcamera2(1),pointcamera2(2), 1.0f };

        cv::Vec3f cameraWorldPos1 = Translation(worldMat1);
        cv::Vec3f cameraWorldPos2 = Translation(worldMat2);

        constexpr float SMALL_NUM = 0.00001f;

        cv::Vec3f u = cv::Vec3f{ worldRay1(0),worldRay1(1),worldRay1(2) } - cameraWorldPos1;
        cv::Vec3f v = cv::Vec3f{ worldRay2(0),worldRay2(1),worldRay2(2) } - cameraWorldPos2;
        cv::Vec3f w = cameraWorldPos1 - cameraWorldPos2;
        float a = u.dot(u); // always >= 0
        float b = u.dot(v);
        float c = v.dot(v); // always >= 0
        float d = u.dot(w);
        float e = v.dot(w);
        float D = a*c - b*b; // always >= 0
        float sc, tc;

        // compute the line parameters of the two closest points
        if (D < SMALL_NUM) { // the lines are almost parallel`
            sc = 0.0;
            tc = (b>c ? d / b : e / c); // use the largest denominator
        }
        else {
            sc = (b*e - c*d) / D;
            tc = (a*e - b*d) / D;
        }

        // get the center of the two closest points
        return (cameraWorldPos1 + (sc * u) +cameraWorldPos2 + (tc * v))*0.5f; // =� (L1(sc) + L2(tc))/2 .... mid point of line
    }
}