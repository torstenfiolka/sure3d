// Software License Agreement (BSD License)
//
// Copyright (c) 2012-2013, Fraunhofer FKIE/US
// All rights reserved.
// Author: Torsten Fiolka
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Fraunhofer FKIE nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SURE_CONFIGURATION_H_
#define SURE_CONFIGURATION_H_

#include <fstream>
#include <ios>
#include <cmath>
#include <vector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>

#include <sure/data/typedef.h>

namespace sure
{
  /**
   * Stores all necessary parameters for calculating SURE features
   */
  class Configuration
  {
    public:

      Configuration()
      {
        Samplingrate = 0.04;
        NormalSamplingrate = 0.02;
        NormalRegionSize = 0.04;
        NormalInfluenceRadius = sure::normal::DEFAULT_NORMAL_HISTOGRAM_INFLUENCE;
        DescriptorSamplingrate = 0.02;
        DescriptorNumberOfDistanceClasses = sure::feature::DEFAULT_NUMBER_OF_DESCRIPTORS;
        MinimumEntropyThreshold = 0.4;
        MinimumCornernessThreshold = 0.15;
        DistanceThreshold = 0.0;
        FeatureSuppressionRatio = 1.0;
        OctreeCenter[0] = OctreeCenter[1] = OctreeCenter[2] = 0.0;
        OctreeSmallestVoxelSize = 0.01;
        OctreeRootVoxelSize = 20.48;
        OctreeMaximumNumberOfNodes = 500000;
        EntropyMode = sure::NORMALS;
        AdditionalPointsOnDepthBorders = true;
        IgnoreNormalsOnBackgroundDepthBorders = false;
        IgnoreBackgroundDetections = true;
        ImproveLocalization = true;
        Scales.push_back(0.12);
        Scales.push_back(0.24);
        Scales.push_back(0.36);
        Scales.push_back(0.48);
      }

      void clearScales() { this->Scales.clear(); }
      Scalar getScale(unsigned index) { return Scales.at(index); }
      const std::vector<Scalar>& getScales() const { return Scales; }
      void addScale(Scalar scale) { Scales.push_back(scale); }

      // Samplingrate for the entropy calculation
      Scalar Samplingrate;

      // Samplingrate for the normal calculation
      Scalar NormalSamplingrate;

      // Size of the region for calculating normals
      Scalar NormalRegionSize;

      // Influence of a single normal in the normal histogram in radians
      Scalar NormalInfluenceRadius;

      // Samplingrate for creating the descriptor
      Scalar DescriptorSamplingrate;

      // Number of distance classes in the descriptor
      Scalar DescriptorNumberOfDistanceClasses;

      // Minimum required entropy value for a feature
      Scalar MinimumEntropyThreshold;

      // Minimum required cornerness value for a feature. A value of zero disables the check
      Scalar MinimumCornernessThreshold;

      // Maximum allowed distance from the sensor for a feature. A value of zero disables the check
      Scalar DistanceThreshold;

      // Specifies the region around a feature in which no other features are allowed. Relative value to the feature size
      Scalar FeatureSuppressionRatio;

      // Center coordinates of the octree
      Scalar OctreeCenter[3];

      // Minimum allowed node edge length for the octree in meters
      Scalar OctreeSmallestVoxelSize;

      // Octree edge length in meters
      Scalar OctreeRootVoxelSize;

      // Number of allocated nodes for the octree.
      int OctreeMaximumNumberOfNodes;

      // Specifies wether normals of cross-products are used for entropy calculation
      EntropyCalculationMode EntropyMode;

      // Additional points along the view direction are added at foreground depth borders
      bool AdditionalPointsOnDepthBorders;

      // Disables normal calculation on background depth borders
      bool IgnoreNormalsOnBackgroundDepthBorders;

      // Disables feature extraction on background depth borders
      bool IgnoreBackgroundDetections;

      // Improves localization with mean shift for found features
      bool ImproveLocalization;

    protected:

      // Stores the scales defining the size of the region for entropy calculation
      std::vector<Scalar> Scales;

    private:

      friend std::ostream& operator<<(std::ostream& stream, const sure::Configuration& config);
      friend class boost::serialization::access;

      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
          ar & Samplingrate;

          ar & NormalSamplingrate;
          ar & NormalRegionSize;
          ar & NormalInfluenceRadius;

          ar & DescriptorSamplingrate;
          ar & DescriptorNumberOfDistanceClasses;

          ar & MinimumEntropyThreshold;
          ar & MinimumCornernessThreshold;

          ar & DistanceThreshold;
          ar & FeatureSuppressionRatio;

          ar & OctreeCenter[0];
          ar & OctreeCenter[1];
          ar & OctreeCenter[2];
          ar & OctreeSmallestVoxelSize;
          ar & OctreeRootVoxelSize;
          ar & OctreeMaximumNumberOfNodes;

          ar & EntropyMode;

          ar & AdditionalPointsOnDepthBorders;
          ar & IgnoreNormalsOnBackgroundDepthBorders;
          ar & IgnoreBackgroundDetections;
          ar & ImproveLocalization;

          ar & Scales;
      }

  };

  std::ostream& operator<<(std::ostream& stream, const sure::Configuration& config);

}


#endif /* CONFIGURATION_H_ */
