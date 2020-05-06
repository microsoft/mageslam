// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "CameraCalibration.h"
#include <iostream>
#include "Utils/cv.h"

namespace mage
{
    CameraCalibration::CameraCalibration() :
        m_cameraMatrix{ cv::Matx33f::zeros() },
        m_invCameraMatrix{ cv::Matx33f::zeros() },
        m_cvDistortionCoeffs{ 0,0,0,0,0,0,0,0 },
        m_width{ 0 },
        m_height{ 0 }
    {

    }

    CameraCalibration::CameraCalibration(std::shared_ptr<const calibration::CameraModel> cameraModel)
    {
        const auto& intrinsics = cameraModel->GetIntrinsics();
        m_cameraMatrix = {
            intrinsics.GetFx(), 0, intrinsics.GetCx(),
            0, intrinsics.GetFy(), intrinsics.GetCy(),
            0, 0, 1
        };

        //PERF: can hardcode inverse
        bool inverted = false;
        m_invCameraMatrix = m_cameraMatrix.inv(cv::DECOMP_LU, &inverted);
        assert(inverted && "Failed to invert matrix");

        m_width = intrinsics.GetCalibrationWidth();
        m_height = intrinsics.GetCalibrationHeight();

        switch (cameraModel->GetDistortionType())
        {
        case calibration::DistortionType::None:
            break;
        case calibration::DistortionType::Poly3k:
        {
            std::shared_ptr<const calibration::Poly3KCameraModel> poly3K = std::dynamic_pointer_cast<const calibration::Poly3KCameraModel>(cameraModel);
            //opencv ordering for distortion terms
            m_cvDistortionCoeffs = { poly3K->GetK1(), poly3K->GetK2(), poly3K->GetP1(),
                poly3K->GetP2(), poly3K->GetK3(), 0, 0, 0 };
        }
        break;
        case calibration::DistortionType::Rational6k:
        {
            std::shared_ptr<const calibration::Rational6KCameraModel> rat6k = std::dynamic_pointer_cast<const calibration::Rational6KCameraModel>(cameraModel);
            m_cvDistortionCoeffs = { rat6k->GetK1(), rat6k->GetK2(), rat6k->GetP1(),
                rat6k->GetP2(), rat6k->GetK3(), rat6k->GetK4(), rat6k->GetK5(), rat6k->GetK6() };
        }
            break;
        default:
            assert(false && "unknown distortion");
        }

        m_distortionType = cameraModel->GetDistortionType();
    }

    std::shared_ptr<const calibration::CameraModel> CameraCalibration::CreateCameraModel() const
    {        
        cv::Vec4f linear{ GetPrincipalPointX(), GetPrincipalPointY(), GetFocalLengthX(), GetFocalLengthY() };
        Intrinsics intrinsics{ gsl::make_span(linear.val), m_width, m_height };

        switch (m_distortionType)
        {
        case calibration::DistortionType::None:
            return std::make_shared<const calibration::PinholeCameraModel>(intrinsics);
        case calibration::DistortionType::Poly3k:
        {
            //note: no longer opencv ordering
            std::vector<float> poly3kDistortion{ GetK1(),GetK2(), GetK3(), GetP1(), GetP2() };
            return std::make_shared<const calibration::Poly3KCameraModel>(intrinsics, poly3kDistortion);
        }
        case calibration::DistortionType::Rational6k:
        {
            //note: no longer opencv ordering
            std::vector<float> rational6kDistortion{ GetK1(),GetK2(), GetK3(), GetK4(), GetK5(), GetK6(), GetP1(), GetP2() };
            return std::make_shared<const calibration::Rational6KCameraModel>(intrinsics, rational6kDistortion);
        }
        default:
            assert(false && "unknown distortion");
            return nullptr;
        }
    }

    bool CameraCalibration::operator==(const CameraCalibration & other) const
    {
        if (GetDistortionType() != other.GetDistortionType())
            return false;

        if (GetCalibrationWidth() != other.GetCalibrationWidth())
            return false;

        if (GetCalibrationHeight() != other.GetCalibrationHeight())
            return false;

        if (GetDistortionType() != calibration::DistortionType::None && !MatEqual(GetCVDistortionCoeffs(), other.GetCVDistortionCoeffs()))
            return false;

        if (!MatxEqual(GetCameraMatrix(),other.GetCameraMatrix()))
            return false;
        
        return true;
    }

    cv::Mat CameraCalibration::GetCVDistortionCoeffs() const
    {
        switch (m_distortionType)
        {
        case calibration::DistortionType::None:
            assert(false && "no calibration coeffs");
            return cv::Mat();
        case calibration::DistortionType::Poly3k:
            return cv::Mat(m_cvDistortionCoeffs.get_minor<5, 1>(0, 0));
        case calibration::DistortionType::Rational6k:
            return cv::Mat(m_cvDistortionCoeffs);
        default:
            assert(false && "unknown calibration");
            return cv::Mat();
        }
    }

    cv::Vec4f CameraCalibration::GetLinearIntrinsics() const
    {
        assert(m_distortionType == mage::calibration::DistortionType::None && "using linear intrinsics on distorted image");
        return { GetPrincipalPointX(), GetPrincipalPointY(), GetFocalLengthX(), GetFocalLengthY() };
    }

    CameraCalibration CameraCalibration::GetScaledCalibration(float scale) const
    {
        Intrinsics scaledIntrinsics = GetScaledIntrinsics(scale);
        switch (m_distortionType)
        {
        case mage::calibration::DistortionType::None:
            return CameraCalibration(std::make_shared<calibration::PinholeCameraModel>(scaledIntrinsics));
        case mage::calibration::DistortionType::Poly3k:
            return CameraCalibration(std::make_shared<calibration::Poly3KCameraModel>(scaledIntrinsics, std::vector<float>{ GetK1(),GetK2(),GetK3(), GetP1(), GetP2()}));
        case mage::calibration::DistortionType::Rational6k:
            return CameraCalibration(std::make_shared<calibration::Rational6KCameraModel>(scaledIntrinsics, std::vector<float>{ GetK1(), GetK2(), GetK3(), GetK4(), GetK5(), GetK6(), GetP1(), GetP2()}));
        default:
            assert(false && "unknown distortion type");
            return CameraCalibration{};
        }
    }

    Intrinsics CameraCalibration::GetScaledIntrinsics(float scale) const
    {
        return { GetPrincipalPointX() * scale,
            GetPrincipalPointY() * scale,
            GetFocalLengthX() * scale,
            GetFocalLengthY() * scale,
            (uint32_t)(m_width* scale), (uint32_t)(m_height * scale) };
    }

    std::ostream& operator <<(std::ostream& stream, const calibration::DistortionType t)
    {
        return stream << mira::underlying_cast(t);
    }

    std::istream& operator >>(std::istream& stream, calibration::DistortionType& t)
    {
        return stream >> mira::underlying_ref_cast(t);
    }
}
