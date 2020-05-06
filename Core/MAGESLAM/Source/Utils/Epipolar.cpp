// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "Epipolar.h"
#include "Data/Pose.h"
#include "Device/CameraCalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace mage
{    
    //imported from Semi-dense prototype
    cv::Matx33f ComputeFundamentalMatrix(const Pose& fromFrame, const CameraCalibration& fromFrameCalibration, const Pose& toFrame, const CameraCalibration& toFrameCalibration)
    {   
        cv::Matx33f essentialMatrix = ComputeEssentialMatrix(fromFrame, toFrame);
        cv::Matx33f invCameraMatrix = fromFrameCalibration.GetInverseCameraMatrix();
        cv::Matx33f invertedAndTransposedCameraMatrix = toFrameCalibration.GetInverseCameraMatrix().t();

        // http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        // see the stereoCalibrate function doc for description of calculation
        cv::Matx33f fundamental = invertedAndTransposedCameraMatrix * essentialMatrix * invCameraMatrix;

        return fundamental;
    }

    //imported from Semi-dense prototype
    //PERF: shouldn't need to extract the rotations/translations, should just be matrix concatenation    
    cv::Matx33f ComputeEssentialMatrix(const Pose& fromFrame, const Pose& toFrame)
    {
        // calculate essential matrix
        cv::Matx34f Mji = fromFrame.GetRelativeViewMatrix(toFrame);
        cv::Matx31f deltaT = Mji.col(3);

        // http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        // see the stereoCalibrate function doc for description of calculation
        // matrix representation of the cross product of translation  (?)
        cv::Matx33f translationMat
        {
                     0,  -deltaT(2),  deltaT(1),
             deltaT(2),          0,  -deltaT(0),
            -deltaT(1),  deltaT(0),           0
        };

        cv::Matx33f rotationMat = Mji.get_minor<3, 3>(0, 0);

        return translationMat * rotationMat;
    }

    // calculate the distance from a point and its corresponding epipolar line
    void DebugRenderEpipolarLine(const cv::Matx33f& f, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2, cv::Mat& frame2)
    {
        // epipolar line is in ax + by + c = 0 (slope intercept) form.
        auto a = f(0, 0) * pointFrame1.x + f(0, 1) * pointFrame1.y + f(0, 2);
        auto b = f(1, 0) * pointFrame1.x + f(1, 1) * pointFrame1.y + f(1, 2);
        auto c = f(2, 0) * pointFrame1.x + f(2, 1) * pointFrame1.y + f(2, 2);

        //render point being tested
        cv::circle(frame2, pointFrame2, 2, CV_RGB(1, 0, 0));

        //find intersection with image borders
        std::vector<cv::Point2f> linePts;

        float y0 = 0;
        float x0 = -c / a;
        if (x0 >= 0 && x0 < frame2.cols)
            linePts.push_back({ x0,y0 });

        float x1 = 0;
        float y1 = -c / b;
        if (y1 >= 0 && y1 < frame2.rows)
            linePts.push_back({ x1, y1 });

        float x2 = (float)frame2.cols - 1.0f;
        float y2 = (a * x2 + c) / -b;
        if(y2 >= 0 && y2 < frame2.rows)
            linePts.push_back({ x2, y2 });

        float y3 = (float)frame2.rows - 1.0f;
        float x3 = (b * y3 + c) / -a;
        if(x3 >= 0 && x3 < frame2.cols)
            linePts.push_back({ x3, y3 });

        //offscreen?
        if (linePts.size() == 0)
            return;

        assert(linePts.size() == 2 && "expecting a epipolar line with a beginning and end on screen");

        cv::line(frame2, linePts[0], linePts[1], CV_RGB(255, 0, 0));
    }

    // calculate the distance from a point and its corresponding epipolar line
    float DistanceFromEpipolarLine(const cv::Matx33f& f, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2)
    {
        auto a = f(0,0) * pointFrame1.x + f(0, 1) * pointFrame1.y + f(0, 2);
        auto b = f(1, 0) * pointFrame1.x + f(1, 1) * pointFrame1.y + f(1, 2);
        auto c = f(2, 0) * pointFrame1.x + f(2, 1) * pointFrame1.y + f(2, 2);

        //todo: do I need to normalize?
        auto nu = a*a + b*b;
        nu = nu ? 1.f / std::sqrt(nu) : 1.f;

        // epipolar line is in ax + by + c = 0 form. substitute in point to formula. should equal zero if on line.
        float error = abs(pointFrame2.x * a + pointFrame2.y * b + c)*nu;
        return error;
    }
    
    // calculate the distance from a point and its corresponding epipolar line
    float DistanceFromEpipolarLineSlow(const cv::Matx33f& fundamentalMat, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2)
    {
        // find epipolar line
        std::vector<cv::Vec3f> line;
        cv::computeCorrespondEpilines(std::vector<cv::Point2f>{ pointFrame1 }, 1, fundamentalMat, line);

        // epipolar line is in ax + by + c = 0 form. substitute in point to formula. should equal zero if on line.
        float error = abs(pointFrame2.x * line[0][0] + pointFrame2.y * line[0][1] + line[0][2]);
        return error;
    }

    //tests if frame1Point is on the epiline created for frame0Point (within distance maxDistance)
    bool IsPointOnEpipolarLine(const cv::Matx33f& fundamentalMat, const cv::Point2f& pointFrame1, const cv::Point2f& pointFrame2, const float errorThresholdPixels)
    {
        float error = DistanceFromEpipolarLine(fundamentalMat, pointFrame1, pointFrame2);
        if (error > errorThresholdPixels)
        {
            return false;
        }

        return true;
    }

}
