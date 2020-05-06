// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Data.h"

#include "arcana\utils\serialization\serializable.h"
#include "arcana\type_traits.h"
#include "Serialization\cv_serialization.h"

#include <opencv2\core\types.hpp>
#include <array>

namespace mage
{
    class CameraCalibration : public mira::serializable<CameraCalibration>
    {
    public:

        CameraCalibration();
        CameraCalibration(std::shared_ptr<const calibration::CameraModel> cameraModel);

        template<typename StreamT, typename = mira::is_stream_t<StreamT>>
        explicit CameraCalibration(StreamT& stream)
        {
            deserialize(stream);
        }

        static constexpr auto members()
        {
            return declare_members(
                &CameraCalibration::m_cameraMatrix,
                &CameraCalibration::m_invCameraMatrix,
                &CameraCalibration::m_cvDistortionCoeffs,
                &CameraCalibration::m_distortionType,
                &CameraCalibration::m_width,
                &CameraCalibration::m_height
            );
        }

        const cv::Matx33f& GetCameraMatrix() const { return m_cameraMatrix; };
        const cv::Matx33f& GetInverseCameraMatrix() const { return m_invCameraMatrix; }

        calibration::DistortionType GetDistortionType() const { return m_distortionType; }
        
        uint32_t GetCalibrationWidth() const { return m_width; }
        uint32_t GetCalibrationHeight() const { return m_height; }

        Intrinsics CameraCalibration::GetScaledIntrinsics(float scale) const;
        CameraCalibration GetScaledCalibration(float scale) const;

        // vector of distortion coefficients in opencv ordering K0 K1 P1 P2 K2 [K3, K4, K5]
        cv::Mat GetCVDistortionCoeffs() const;
       
        // vector of the linear intrinsics CX CY FX FY
        cv::Vec4f GetLinearIntrinsics() const;
       
        float GetFocalLengthX() const { return m_cameraMatrix(0, 0); }
        float GetFocalLengthY() const { return m_cameraMatrix(1, 1); }
        float GetPrincipalPointX() const { return m_cameraMatrix(0, 2); }
        float GetPrincipalPointY() const { return m_cameraMatrix(1, 2); }

        float GetK1() const { return m_cvDistortionCoeffs(0); }     //radial distortion coefficient R^2
        float GetK2() const { return m_cvDistortionCoeffs(1); }     //radial distortion coefficient R^4
        float GetP1() const { return m_cvDistortionCoeffs(2); }     //tangential distortion coefficient
        float GetP2() const { return m_cvDistortionCoeffs(3); }     //tangential distortion coefficient
        float GetK3() const { return m_cvDistortionCoeffs(4); }     //radial distortion coefficient R^6
        float GetK4() const { assert(m_distortionType == calibration::DistortionType::Rational6k && "invalid distortion term"); return m_cvDistortionCoeffs(5); }     //radial distortion coefficient for rational 6k
        float GetK5() const { assert(m_distortionType == calibration::DistortionType::Rational6k && "invalid distortion term"); return m_cvDistortionCoeffs(6); }     //radial distortion coefficient for rational 6k
        float GetK6() const { assert(m_distortionType == calibration::DistortionType::Rational6k && "invalid distortion term"); return m_cvDistortionCoeffs(7); }     //radial distortion coefficient for rational 6k

        std::shared_ptr<const calibration::CameraModel> CreateCameraModel() const;

        bool operator==(const CameraCalibration& other) const;
        bool operator!=(const CameraCalibration& other) const { return !(*this == other); }
  
       private:

        cv::Matx33f m_cameraMatrix;
        cv::Matx33f m_invCameraMatrix;
        cv::Matx<float, 8, 1> m_cvDistortionCoeffs;       // vector of opencv distortion coefficients K1 K2 P1 P2 K3 K4 K5 K6
        calibration::DistortionType m_distortionType;
        uint32_t m_width;
        uint32_t m_height;
    };

    std::ostream& operator <<(std::ostream& stream, const calibration::DistortionType t);
    std::istream& operator >>(std::istream& stream, calibration::DistortionType& t);
}
