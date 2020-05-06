// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <math.h>

namespace mage
{
    //distance: the distance you are querying for
    //dmin: the minimum distance at which a point can be observed
    //scale factor: the difference in scale between octave levels 
    inline int ComputeOctave(float distance, float dmin, float scaleFactor)
    {
        return static_cast<int>(roundf(log2f(distance / dmin) / log2f(scaleFactor) - 0.5f));
    }

    // ********************************************************
    // devnote regarding offsetting the octave by 1/2 a unit
    // The best octave matches are the ones 'near' the target octave
    // To model this, we want to compute the distances surrounding the octave
    // rather than bookended by two different octaves.  So, when computing
    // the DMin and DMax, we use the distances generated at the midpoints between
    // the octave and its neighbor.
    //
    // So, for a feature expected to be on octave 4, culling should operate
    // outside the range (3.5, 4.5), not (4, 5).  For this reason, we add
    // an imaginary "half octave" to the expectated octave when computing the
    // boundaries.
    // ********************************************************

    inline float ComputeDMax(float distance, int octave, int maxOctave, float scaleFactor)
    {
        return distance * powf(scaleFactor, static_cast<float>(maxOctave - (octave + 0.5f)));
    }

    inline float ComputeDMin(float distance, int octave, float scaleFactor)
    {
        return distance * powf(scaleFactor, 0.f - (octave + 0.5f));
    }

    inline float MapPointRefinementConfidence(const uint64_t& refinementCount)
    {
        // The intent of the function is to weight map points confidence according to how many times they have been refined.
        // Implicitly, this assumes that the map points are more acurate after several refinement steps
        // This function is also built upon the observation that map points typically settle into the map after 4-5 local bundle adjust iterations
        // So, this function should approach one with an input of 5 refinements.
        return 1.f - 1.f / powf((1.5f + refinementCount), 2.0f);
    }
}
