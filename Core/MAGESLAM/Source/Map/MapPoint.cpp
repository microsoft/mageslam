// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//------------------------------------------------------------------------------
// MapPoint.cpp
//
// Mappoints are 3D points that have been estimated from triangulating
// the positions of the same 2D image space feature viewed from multiple images
//------------------------------------------------------------------------------

#include "MageSettings.h"
#include "MapPoint.h"
#include "Tracking\FeatureMatcher.h"
#include "MappingMath.h"

#include "Data\Keyframe.h"
#include "Utils\cv.h"

#include <assert.h>

namespace mage
{
    MapPoint::MapPoint(const Id<MapPoint>& id, size_t numLevels)
        : m_id{ id },
        m_octaveCounters(numLevels, 0)
    {
    }

    /// accessor for keyframes that have a feature determined to be this map point
    const std::vector<Keyframe const*>& MapPoint::GetKeyframes() const
    {
        return m_keyframes;
    }

    void MapPoint::AddKeyframeAssociation(Keyframe const* keyframe, KeypointDescriptorIndex keypointIndex)
    {
        assert(find(m_keyframes.begin(), m_keyframes.end(), keyframe) == m_keyframes.end());
        m_keyframes.push_back(keyframe);

        int keypointOctave = keyframe->GetKeyPoint(keypointIndex).octave;
        assert(static_cast<size_t>(keypointOctave) < m_octaveCounters.size());
        m_octaveCounters[keypointOctave]++;

        UpdateRepresentativeDescriptor();
        UpdateMeanViewDirectionAndDistances();
    }

    void MapPoint::RemoveKeyframeAssociation(Keyframe const* keyframe, KeypointDescriptorIndex removedDescriptorIndex)
    {
        auto itr = find(m_keyframes.begin(), m_keyframes.end(), keyframe);
        assert(itr != m_keyframes.end());
        m_keyframes.erase(itr);

        int keypointOctave = keyframe->GetKeyPoint(removedDescriptorIndex).octave;

        m_octaveCounters[keypointOctave]--;
        assert(m_octaveCounters[keypointOctave] >= 0);

        UpdateRepresentativeDescriptor();
        UpdateMeanViewDirectionAndDistances();
    }

    const size_t MapPoint::GetKeyPointCountAtLevel(size_t level) const
    {
        assert(level < m_octaveCounters.size());
        return m_octaveCounters[level];
    }

    void MapPoint::ClearAssociations()
    {
        m_keyframes.clear();

        // set all the values to 0
        std::fill(m_octaveCounters.begin(), m_octaveCounters.end(), 0);

        UpdateRepresentativeDescriptor();
        UpdateMeanViewDirectionAndDistances();
    }

    void MapPoint::UpdateRepresentativeDescriptor()
    {
        m_representativeDescriptorKeyframe = nullptr;
        m_representativeAssociation = (KeypointDescriptorIndex)-1;

        if (m_keyframes.size() == 0)
        {            
            return;
        }

        if (m_keyframes.size() == 1)
        {
            m_representativeDescriptorKeyframe = m_keyframes[0];
            m_representativeAssociation = m_representativeDescriptorKeyframe->GetAssociatedIndex(this);
            return;
        }

        //get full list of features    
        std::vector<KeypointDescriptorIndex> associationIndexes;
        associationIndexes.reserve(m_keyframes.size());
        for (const Keyframe* keyframe : m_keyframes)
        {
            associationIndexes.push_back(keyframe->GetAssociatedIndex(this));
        }
        
        float leastSummedDistance = std::numeric_limits<float>::max();
        int representativeKeyframeIdx = -1;
        for (size_t kfIndex = 0; kfIndex < m_keyframes.size(); ++kfIndex)
        {
            const auto& kfDescriptor = m_keyframes[kfIndex]->GetAnalyzedImage()->GetDescriptor(associationIndexes[kfIndex]);

            //compare to all the others
            float curSummedDistances = 0;
            for (size_t otherKfIndex = 0; otherKfIndex < m_keyframes.size(); ++otherKfIndex)
            {
                curSummedDistances += GetDescriptorDistance(
                    kfDescriptor,
                    m_keyframes[otherKfIndex]->GetAnalyzedImage()->GetDescriptor(associationIndexes[otherKfIndex]));
            }

            if (curSummedDistances < leastSummedDistance)
            {
                representativeKeyframeIdx = gsl::narrow<int>(kfIndex);
                leastSummedDistance = curSummedDistances;
            }
        }

        assert(representativeKeyframeIdx != -1);
        m_representativeDescriptorKeyframe = m_keyframes[representativeKeyframeIdx];
        m_representativeAssociation = associationIndexes[representativeKeyframeIdx];
    }

    void MapPoint::UpdateMeanViewDirectionAndDistances()
    {
        m_meanViewingDirection = cv::Vec3f(0.0f, 0.0f, 0.0f);

        if (m_keyframes.size() == 0)
        {
            return;
        }
                
        for (auto* curKeyframe : m_keyframes)
        {
            cv::Vec3f viewDir = CalculateDeltaWorldPositionFromKeyframe(curKeyframe);
            m_meanViewingDirection += Normalize(viewDir);
        }
        m_meanViewingDirection = Normalize(m_meanViewingDirection);

        auto deltaPosition = m_representativeDescriptorKeyframe->GetPose().GetWorldSpacePosition() - m_position;
        auto distance = sqrtf(deltaPosition.dot(deltaPosition));

        auto currentOctave = m_representativeDescriptorKeyframe->GetAnalyzedImage()->GetKeyPoint(m_representativeAssociation).octave;

        m_dMax = ComputeDMax(distance, currentOctave, gsl::narrow_cast<int>(m_representativeDescriptorKeyframe->GetAnalyzedImage()->GetNumLevels()), m_representativeDescriptorKeyframe->GetAnalyzedImage()->GetPyramidScale());
        m_dMin = ComputeDMin(distance, currentOctave, m_representativeDescriptorKeyframe->GetAnalyzedImage()->GetPyramidScale());
    }

    cv::Vec3f MapPoint::CalculateDeltaWorldPositionFromKeyframe(const Keyframe* keyframe)
    {
        assert(keyframe != nullptr);

        return{ GetPosition() - keyframe->GetPose().GetWorldSpacePosition() };
    }

    const ORBDescriptor& MapPoint::GetRepresentativeDescriptor() const
    {
        assert(m_representativeDescriptorKeyframe != nullptr);
        return m_representativeDescriptorKeyframe->GetAnalyzedImage()->GetDescriptor(m_representativeAssociation);
    }

    void MapPoint::SetPosition(const cv::Point3f position)
    {
        m_position.x = position.x;
        m_position.y = position.y;
        m_position.z = position.z;

        UpdateMeanViewDirectionAndDistances();
    }

    const cv::Point3f& MapPoint::GetPosition() const
    {
        return m_position;
    }
}
