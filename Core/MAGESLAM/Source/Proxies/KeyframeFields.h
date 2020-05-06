// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "Data\Types.h"
#include "Data\Keyframe.h"

#include "arcana\utils\serialization\serializable.h"

namespace UnitTests
{
    class LocalBundleAdjustmentUnitTests;
}

namespace mage
{
    namespace proxy
    {
        /*
            Represents the image property of a Keyframe
        */
        struct Image
        {
            explicit Image(const Keyframe& original)
                : m_prop{ original.GetAnalyzedImage() }
            {}

            explicit Image(const std::shared_ptr<const AnalyzedImage>& image)
                : m_prop{ image }
            {}

            explicit Image(const Image& image)
                : m_prop{ image.m_prop }
            {}

            const std::shared_ptr<const AnalyzedImage>& GetAnalyzedImage() const
            {
                return m_prop;
            }
        private:
            std::shared_ptr<const AnalyzedImage> m_prop;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const Image& proxy)
        {
            intro(
                cereal::make_nvp("AnalyzedImage", proxy.GetAnalyzedImage())
            );
        }

        struct Intrinsics : mira::serializable<Intrinsics>
        {
            explicit Intrinsics(const Keyframe& original)
                : m_undistortedIntrinsics{ original.GetUndistortedIntrinsics() }
            {}

            explicit Intrinsics(const mage::Intrinsics& intrinsics)
                : m_undistortedIntrinsics{ intrinsics }
            {}

            explicit Intrinsics(const Intrinsics& intrinsics)
                : m_undistortedIntrinsics{ intrinsics.m_undistortedIntrinsics }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            explicit Intrinsics(StreamT& stream)
            {
                deserialize(stream);
            }

            static constexpr auto members()
            {
                return declare_members(
                    &Intrinsics::m_undistortedIntrinsics
                );
            }

            const mage::Intrinsics& GetUndistortedIntrinsics() const
            {
                return m_undistortedIntrinsics;
            }

            void SetUndistortedIntrinsics(const mage::Intrinsics& intrinsics)
            {
                m_undistortedIntrinsics = intrinsics;
            }
        private:
            mage::Intrinsics m_undistortedIntrinsics;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const Intrinsics& proxy)
        {
            intro(
                cereal::make_nvp("UndistortedIntrinsics", proxy.GetUndistortedIntrinsics())
            );
        }

        /*
            Represents the Pose property of a Keyframe
        */
        struct Pose : mira::serializable<Pose>
        {
            Pose() = default;

            explicit Pose(const Keyframe& original)
                : m_pos{ original.GetPose() }
            {}

            explicit Pose(const mage::Pose& pose)
                : m_pos{ pose }
            {}

            explicit Pose(const Pose& pose)
                : m_pos{ pose.m_pos }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            explicit Pose(StreamT& stream)
            {
                deserialize(stream);
            }

            static constexpr auto members()
            {
                return declare_members(
                    &Pose::m_pos
                );
            }

            const mage::Pose& GetPose() const
            {
                return m_pos;
            }

            void SetPose(const mage::Pose& pose)
            {
                m_pos = pose;
            }
        private:
            mage::Pose m_pos;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const Pose& proxy)
        {
            intro(
                cereal::make_nvp("Pose", proxy.GetPose())
            );
        }

        /*
            Represents the constraints placed on the Keyframe during bundle adjust
        */
        struct PoseConstraints
        {
            explicit PoseConstraints(const Keyframe& original)
                : m_immortal{ original.IsImmortal() }, m_fixed { original.IsFixed() }, m_tethers{ original.GetTethers() }
            {}

            explicit PoseConstraints(const PoseConstraints& other)
                : m_immortal{ other.m_immortal }, m_fixed { other.m_fixed }, m_tethers{ other.m_tethers }
            {}

            template<typename StreamT, typename = mira::is_stream_t<StreamT>>
            explicit PoseConstraints(StreamT& stream)
            {
                deserialize(stream);
            }

            explicit PoseConstraints(bool fixed = false)
                : m_fixed{ fixed }, m_tethers{}
            {}

            const bool IsImmortal() const
            {
                return m_immortal;
            }

            void IsImmortal(bool sacrosanct)
            {
                m_immortal = sacrosanct;
            }

            const bool IsFixed() const
            {
                return m_fixed;
            }

            void SetFixed(bool fixed)
            {
                m_fixed = fixed;
            }

            const std::vector<Tether<Keyframe>>& GetTethers() const
            {
                return m_tethers;
            }

        protected:
            void AddDistanceTether(const mage::Id<Keyframe>& originId, float distance, float weight)
            {
                m_tethers.emplace_back(originId, distance, weight);
            }

            void AddExtrinsicTether(const mage::Id<Keyframe>& originId, const cv::Vec3f& position, const Quaternion& rotation, float weight)
            {
                m_tethers.emplace_back(originId, position, rotation, weight, TetherType::EXTRINSIC);
            }

            void AddRotationTether(const mage::Id<Keyframe>& originId, const Quaternion& rotation, float weight)
            {
                m_tethers.emplace_back(originId, rotation, weight);
            }

        private:
            bool m_immortal{};
            bool m_fixed{};
            std::vector<Tether<Keyframe>> m_tethers;

            friend class ::UnitTests::LocalBundleAdjustmentUnitTests;
        };

        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const PoseConstraints& proxy)
        {
            intro(
                cereal::make_nvp("IsImmortal", proxy.IsImmortal()),
                cereal::make_nvp("IsFixed", proxy.IsFixed()),
                cereal::make_nvp("Tethers", proxy.GetTethers())
            );
        }

        /*
            Represents the MapPoint associations contained in a Keyframe
        */
        template<typename MapPointProxyT>
        struct Associations
        {
            using MapPointT = MapPointProxyT;

            Associations(
                const std::shared_ptr<const AnalyzedImage>& image,
                const std::vector<typename MapPointAssociations<MapPointT>::Association>& associations)
                : m_mapPoints{ image, associations }
            {
#ifndef NDEBUG
                std::set<mage::Id<MapPoint>> mapPointSet;
                std::set<KeypointDescriptorIndex> keypointSet;

                for (const auto& association : associations)
                {
                    assert(mapPointSet.insert(association.MapPoint.GetId()).second);
                    assert(keypointSet.insert(association.Index).second);
                }
#endif
            }

            explicit Associations(const std::shared_ptr<const AnalyzedImage>& image)
                : m_mapPoints{ image }
            {}

            size_t GetMapPointCount() const
            {
                return m_mapPoints.GetMapPointCount();
            }

            void GetMapPoints(std::vector<MapPointT>& points) const
            {
                m_mapPoints.GetMapPoints(points);
            }

            void GetMapPoints(temp::vector<MapPointT>& points) const
            {
                m_mapPoints.GetMapPoints(points);
            }

            void GetMapPoints(loop::vector<MapPointT>& points) const
            {
                m_mapPoints.GetMapPoints(points);
            }

            bool HasMapPoint(const mage::Id<MapPoint>& id) const
            {
                return m_mapPoints.HasMapPoint(id);
            }

            const cv::KeyPoint& GetAssociatedKeyPoint(const MapPointT& mpp) const
            {
                return m_mapPoints.GetAssociatedKeyPoint(mpp);
            }

            KeypointDescriptorIndex GetAssociatedIndex(const MapPointT& mappoint) const
            {
                return m_mapPoints.GetAssociatedIndex(mappoint);
            }

            KeypointDescriptorIndex GetAssociatedIndex(const mage::Id<MapPoint>& id) const
            {
                return m_mapPoints.GetAssociatedIndex(id);
            }

            const MapPointT* GetAssociatedMapPoint(KeypointDescriptorIndex index) const
            {
                return m_mapPoints.GetAssociatedMapPoint(index);
            }

            void GetAssociations(std::vector<typename MapPointAssociations<MapPointT>::Association>& associations) const
            {
                return m_mapPoints.GetAssociations(associations);
            }

            void IterateAssociations(const std::function<void(const MapPointT&, KeypointDescriptorIndex)>& iterator) const
            {
                return m_mapPoints.IterateAssociations(iterator);
            }

            void IterateAssociations(const std::function<void(MapPointT&, KeypointDescriptorIndex)>& iterator)
            {
                return m_mapPoints.IterateAssociations(iterator);
            }

            void AddAssociation(const MapPointT& mapPoint, KeypointDescriptorIndex association)
            {
                m_mapPoints.AddAssociation(mapPoint, association);
            }

            void RemoveAssociation(const MapPointT& mapPoint)
            {
                m_mapPoints.RemoveAssociation(mapPoint);
            }

            void RemoveAssociation(const mage::Id<MapPoint>& id)
            {
                m_mapPoints.RemoveAssociation(id);
            }

            void ClearAssociations()
            {
                m_mapPoints.ClearAssociations();
            }

            bool TryRemoveAssociation(const mage::Id<MapPoint>& id)
            {
                return m_mapPoints.TryRemoveAssociation(id);
            }

            const size_t GetUnassociatedKeypointCount() const
            {
                return m_mapPoints.GetUnassociatedCount();
            }

            std::vector<bool> GetUnassociatedKeypointMask() const
            {
                return m_mapPoints.CreateUnassociatedMask();
            }

            temp::vector<bool> GetUnassociatedKeypointMask(thread_memory& memory) const
            {
                return m_mapPoints.CreateUnassociatedMask(memory);
            }

            const size_t GetAssociatedKeypointCount() const
            {
                return m_mapPoints.GetAssociatedCount();
            }

            std::vector<bool> GetAssociatedKeypointMask() const
            {
                return m_mapPoints.CreateAssociatedMask();
            }

            size_t GetSharedMapPointCount(const Associations<MapPointT>& otherAssociations) const
            {
                return m_mapPoints.GetSharedMapPointCount(otherAssociations.m_mapPoints);
            }

            template<typename T>
            MapPointAssociations<T> ConvertAssociations() const
            {
                return m_mapPoints.Convert<T>();
            }

            void OverwriteAssociations(MapPointAssociations<MapPointT>&& assocs)
            {
                m_mapPoints = std::move(assocs);
            }
        protected:
            explicit Associations(const Keyframe& original)
                : m_mapPoints{ original.ConvertAssociations<MapPointT>() }
            {}

        private:
            MapPointAssociations<MapPointT> m_mapPoints;
        };

        template<typename ArchiveT, typename MapPointProxyT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const Associations<MapPointProxyT>& proxy)
        {
            proxy.IterateAssociations([&intro](const MapPointProxyT& mp, KeypointDescriptorIndex idx)
            {
                intro(
                    cereal::make_nvp("MapPoint", mp.GetId()),
                    cereal::make_nvp("Index", idx)
                );
            });
        }

        /*
            Represents the MapPoint associations contained in a Keyframe
        */
        struct UnAssociatedMask
        {
            const size_t GetUnassociatedKeypointCount() const
            {
                return m_unassociatedCount;
            }

            const std::vector<bool>& GetUnassociatedKeypointMask() const
            {
                return m_unassociatedMask;
            }

            void SetKeypointAsAssociated(size_t index)
            {
                m_unassociatedMask[index] = false;
            }

            template<typename MapPointT>
            explicit UnAssociatedMask(const Associations<MapPointT>& associations)
                : m_unassociatedMask{ associations.GetUnassociatedKeypointMask() },
                m_unassociatedCount{ associations.GetUnassociatedKeypointCount() }
            {}

        protected:
            explicit UnAssociatedMask(const Keyframe& original)
                :   m_unassociatedMask{ original.CreateUnassociatedMask() },
                    m_unassociatedCount{ original.GetUnassociatedCount() }
            {}

        private:
            std::vector<bool> m_unassociatedMask;
            size_t m_unassociatedCount;
        };


        template<typename ArchiveT>
        void introspect_object(mira::introspector<ArchiveT>& intro, const UnAssociatedMask& object)
        {
            intro(
                cereal::make_nvp("UnassociatedKeypointCount", object.GetUnassociatedKeypointCount())
            );
        }

    }
}
