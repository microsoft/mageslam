// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <algorithm>
#include <array>
#include <vector>
#include <gsl/gsl>
#include <assert.h>
#include <boost/optional.hpp>
#include "Data/Intrinsics.h"
#include "TimeUnits.h"
#include "MageSettings.h"

namespace mage
{
    enum class TrackingState
    {
        INITIALIZING = 0,
        TRACKING = 1,
        RELOCALIZING = 2,
        SKIPPED = 3
    };

    enum class FuserMode
    {
        Invalid = 0 ,
        WaitForMageInit = 1,
        WaitForGravityConverge = 2,
        VisualTrackingLost = 3,
        VisualTrackingReacquired = 4,
        ScaleInit = 5,
        Tracking = 6
    };

    enum class PixelFormat
    {
        GRAYSCALE8,
        NV12
    };

    struct FrameId
    {
        std::uint64_t CorrelationId;    //this is the number used to match frames either from each stereo cameras, or from low-res to high-res in the mono case. Should be unique (and increasing) per camera.
        CameraIdentity Camera;

        FrameId()
            : CorrelationId{ 0 }, Camera{ mage::CameraIdentity::MONO }
        {}

        FrameId(const std::uint64_t& id, CameraIdentity cam)
            : CorrelationId{ id }, Camera{ cam }
        {}

        bool operator ==(const FrameId& other) const
        {
            return CorrelationId == other.CorrelationId && Camera == other.Camera;
        }

        bool operator !=(const FrameId& other) const
        {
            return !(*this == other);
        }

        bool operator <(const FrameId& other) const
        {
            if (CorrelationId < other.CorrelationId)
                return true;

            if (other.CorrelationId < CorrelationId)
                return false;

            return Camera < other.Camera;
        }
    };

    struct Size
    {
        size_t Width;
        size_t Height;
    };

    struct Rect
    {
        int X;
        int Y;
        size_t Width;
        size_t Height;
    };

    struct Matrix
    {
        float M11, M12, M13, M14;
        float M21, M22, M23, M24;
        float M31, M32, M33, M34;
        float M41, M42, M43, M44;
    };

    struct Position
    {
        float X;
        float Y;
        float Z;

        std::array<float, 3> AsArray() const
        {
            return{ X, Y, Z };
        }
    };

    struct ProjectedPoint
    {
        float X;
        float Y;
        float Depth;
        int32_t Id;

        ProjectedPoint(float x, float y, float depth, int32_t id)
            : X{ x }, Y{ y }, Depth{ depth }, Id{ id }
        {}

        ProjectedPoint() = default;
    };

    struct AxisAlignedVolume
    {
        Position Min;
        Position Max;
    };
    
    struct Direction
    {
        float X;
        float Y;
        float Z;
    };

    enum class DataFormat
    {
        BINARY,
        PNG,
        LIVE,
        BOB,
        MIDDLEBURY,
    };

    namespace calibration
    {
        enum class DistortionType
        {
            None,
            Poly3k,
            Rational6k
        };

        class CameraModel
        {
        public:

            CameraModel() = default;
            CameraModel(const Intrinsics& intrinsics, DistortionType distType) : m_intrinsics(intrinsics), m_distortionType(distType) {}
            const Intrinsics& GetIntrinsics() const { return m_intrinsics; }
           
            //not in opencv order
            virtual gsl::span<const float> GetDistortionCoefficients() const = 0;
            virtual DistortionType GetDistortionType() const { return m_distortionType; }

        private:
            Intrinsics m_intrinsics;
            DistortionType m_distortionType;
        };

        class PinholeCameraModel : public CameraModel
        {
        public:
            PinholeCameraModel() = delete;
            PinholeCameraModel(const Intrinsics& intrinsics) : CameraModel(intrinsics, DistortionType::None) {}

            gsl::span<const float> GetDistortionCoefficients() const { return {}; }
        };

        class Poly3KCameraModel : public CameraModel
        {
        public:
            Poly3KCameraModel() = delete;
            Poly3KCameraModel(const Intrinsics& intrinsics, std::vector<float> distortionCoefficients) :
                CameraModel(intrinsics, DistortionType::Poly3k),
                m_distortionCoefficients{ distortionCoefficients }
            {
                assert(m_distortionCoefficients.size()== 5 && "incorrect number of distortion coefficients. want k1, k2, k3, p1, p2");
            }

            //not in opencv order
            gsl::span<const float> GetDistortionCoefficients() const { return m_distortionCoefficients; }
          
            float GetK1() const { return m_distortionCoefficients[0];}
            float GetK2() const { return m_distortionCoefficients[1];}
            float GetK3() const { return m_distortionCoefficients[2];}
            float GetP1() const { return m_distortionCoefficients[3];}
            float GetP2() const { return m_distortionCoefficients[4];}
            
            void SetK1(float k1) { m_distortionCoefficients[0] = k1; }
            void SetK2(float k2) { m_distortionCoefficients[1] = k2; }
            void SetK3(float k3) { m_distortionCoefficients[2] = k3; }
            void SetP1(float p1) { m_distortionCoefficients[3] = p1; }
            void SetP2(float p2) { m_distortionCoefficients[4] = p2; }

        private:
            // k1, k2, k3, p1, p2
            std::vector<float> m_distortionCoefficients;
        };

        class Rational6KCameraModel : public CameraModel
        {
        public:
            Rational6KCameraModel() = delete;
            Rational6KCameraModel(const Intrinsics& intrinsics, std::vector<float> distortionCoefficients)
                : CameraModel(intrinsics, DistortionType::Rational6k),
                m_distortionCoefficients{ distortionCoefficients }
            {
                assert(m_distortionCoefficients.size() == 8 && "incorrect number of distortion coefficients. want k1, k2, k3, k4, k5, k6, p1, p2");
            }

            //not in opencv order
            gsl::span<const float> GetDistortionCoefficients() const { return m_distortionCoefficients; }

            float GetK1() const { return m_distortionCoefficients[0]; }
            float GetK2() const { return m_distortionCoefficients[1]; }
            float GetK3() const { return m_distortionCoefficients[2]; }
            float GetK4() const { return m_distortionCoefficients[3]; }
            float GetK5() const { return m_distortionCoefficients[4]; }
            float GetK6() const { return m_distortionCoefficients[5]; }
            float GetP1() const { return m_distortionCoefficients[6]; }
            float GetP2() const { return m_distortionCoefficients[7]; }

            void SetK1(float k1) { m_distortionCoefficients[0] = k1; }
            void SetK2(float k2) { m_distortionCoefficients[1] = k2; }
            void SetK3(float k3) { m_distortionCoefficients[2] = k3; }
            void SetK4(float k4) { m_distortionCoefficients[3] = k4; }
            void SetK5(float k5) { m_distortionCoefficients[4] = k5; }
            void SetK6(float k6) { m_distortionCoefficients[5] = k6; }
            void SetP1(float p1) { m_distortionCoefficients[6] = p1; }
            void SetP2(float p2) { m_distortionCoefficients[7] = p2; }

        private:
            // k1, k2, k3, k4, k5, k6, p1, p2
            std::vector<float> m_distortionCoefficients;
        };

        struct Line
        {
            float M;
            float B;
        };

        struct Bounds
        {
            float Lower;
            float Upper;
        };
        
        class LinearFocalLengthModel
        {
        public:

            LinearFocalLengthModel(Line fx, Line fy, float cx, float cy, Bounds focalbounds, Size calibrationSize,
                const std::vector<float>& distortionPoly3k = {},
                const std::vector<float>& distortionRational6k = {}) :
                m_fx(fx), m_fy(fy), m_cx(cx), m_cy(cy), m_focalBounds(focalbounds), m_calibrationSize(calibrationSize)
            {
                assert(distortionPoly3k.empty() || distortionRational6k.empty() && "can't pass two types currently. calibrations have different intrinsics depending on model");
                assert(cx >= 0 && cx <= 1 && "cx and cy need to be a ratio");
                assert(cy >= 0 && cy <= 1 && "cx and cy need to be a ratio");

                if (!distortionPoly3k.empty())
                {
                    assert(distortionPoly3k.size() == 5 && "expecting k0, k1, k2, p0, p1, for poly3k distortion");
                    std::copy(distortionPoly3k.begin(), distortionPoly3k.end(), std::back_inserter(m_distortionPoly3k));
                }

                if (!distortionRational6k.empty())
                {
                    assert(distortionRational6k.size() == 8 && "expecting k0, k1, k2, k3, k4, k5, p0, p1, for rational6k distortion");
                    std::copy(distortionRational6k.begin(), distortionRational6k.end(), std::back_inserter(m_distortionRational6k));
                }
            }

            LinearFocalLengthModel()
            {
                m_fx = { 0,0 };
                m_fy = { 0,0 };
                m_cx = 0;
                m_cy = 0;
                m_focalBounds = { 0,0 };
                m_calibrationSize = { 0, 0 };
            }

            Intrinsics CreateIntrinsics(boost::optional<uint32_t> lensPosition, size_t width, size_t height) const
            {
                assert(width / (float)height == m_calibrationSize.Width / (float)m_calibrationSize.Height && "aspect not equal need to crop to modify resolution");
                assert(lensPosition || (m_fx.M == 0 && m_fy.M == 0));

                float fx = m_fx.B;
                float fy = m_fy.B;

                if (lensPosition)
                {
                    fx += m_fx.M * lensPosition.value();
                    fy += m_fy.M * lensPosition.value();
                }

                float FxInPixelCoordinates = fx * width;
                float FyInPixelCoordinates = fy * height;
                float CxInPixelCoordinates = m_cx * width;
                float CyInPixelCoordinates = m_cy * height;

                return Intrinsics(CxInPixelCoordinates, CyInPixelCoordinates,
                    FxInPixelCoordinates, FyInPixelCoordinates,
                    gsl::narrow<uint32_t>(width), gsl::narrow<uint32_t>(height));
            }

            std::shared_ptr<const PinholeCameraModel> CreatePinholeCameraModel(const boost::optional<uint32_t>& lensPosition, size_t width, size_t height) const
            {
                return std::make_shared<const PinholeCameraModel>(CreateIntrinsics(lensPosition, width, height));
            }

            std::shared_ptr<const Poly3KCameraModel> CreatePoly3kCameraModel(const boost::optional<uint32_t>& lensPosition, size_t width, size_t height) const
            {
                return std::make_shared<const Poly3KCameraModel>(CreateIntrinsics(lensPosition, width, height), m_distortionPoly3k);
            }

            std::shared_ptr<const Rational6KCameraModel> CreateRational6kCameraModel(const boost::optional<uint32_t>& lensPosition, size_t width, size_t height) const
            {
                return std::make_shared<const Rational6KCameraModel>(CreateIntrinsics(lensPosition, width, height), m_distortionRational6k);
            }


            std::shared_ptr<const CameraModel> GetCameraModel(const boost::optional<uint32_t>& lensPosition, size_t width, size_t height)
            {
                if (HasPoly3kModel())
                {
                    return CreatePoly3kCameraModel(lensPosition, width, height);
                }
                else if (HasRational6kModel())
                {
                    return CreateRational6kCameraModel(lensPosition, width, height);
                }
                else
                {
                    return CreatePinholeCameraModel(lensPosition, width, height);
                }
            }

            Line GetFx() const { return m_fx; }
            Line GetFy() const { return m_fy; }
            float GetCx() const { return m_cx; }
            float GetCy() const { return m_cy; }
            Bounds GetFocalBounds() const { return m_focalBounds; }
            Size GetCalibrationSize() const { return m_calibrationSize; }

            bool HasPoly3kModel()
            {
                return m_distortionPoly3k.size() > 0;
            }

            bool HasRational6kModel()
            {
                return m_distortionRational6k.size() > 0;
            }

            std::vector<float> GetDistortionPoly3k() const { return m_distortionPoly3k; }
            std::vector<float> GetDistortionRational6k() const { return m_distortionRational6k; }

        private:
            //F = m*focusValue + b
            Line m_fx, m_fy;
            float m_cx, m_cy;
            Bounds m_focalBounds;
            Size m_calibrationSize;

            //distortion
            std::vector<float> m_distortionPoly3k;
            std::vector<float> m_distortionRational6k;
        };
    }

    struct Depth
    {
        static constexpr float INVALID_DEPTH = -1.0f;

        float NearPlaneDepth;
        float FarPlaneDepth;
        gsl::span<const ProjectedPoint> SparseDepth;

        Depth(float nearDepth, float farDepth, gsl::span<const ProjectedPoint> sparse)
            : NearPlaneDepth{ nearDepth }, FarPlaneDepth{ farDepth }, SparseDepth{ sparse }
        {}

        Depth()
            : NearPlaneDepth{ INVALID_DEPTH }, FarPlaneDepth{ INVALID_DEPTH }
        {}
    };
}
