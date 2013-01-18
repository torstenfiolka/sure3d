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

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <fstream>
#include <iostream>
#include <ios>
#include <cmath>
#include <sstream>

namespace sure {

enum EntropyCalculationMode
{
  NORMALS = 0,
  CROSSPRODUCTS_ALL_NORMALS_WITH_MAIN_NORMAL,
  CROSSPRODUCTS_ALL_NORMALS_PAIRWISE
};

enum CrossProductWeightMethod
{
  NO_WEIGHT = 0,
  INVERSE_ABSOLUTE_DOT_PRODUCT,
  INVERSE_POSITIVE_DOT_PRODUCT,
  EXCLUSION
};

//! stores configuration data
class Configuration
{
public:


  Configuration()
  {
    reset();
  }

  void reset();

  void setSamplingRate(float rate) { this->samplingRate = rate; this->samplingLevel = this->getSamplingMapIndex(rate); }
  void setSize(float size) { this->histogramSize = size; this->histogramRadius = size*0.5f; this->featureInfluenceRadius = size; }
  void setNormalsScale(float scale) { this->normalScale = scale; this->normalScaleRadius = scale*0.5f; }
  void setNormalSamplingRate(float rate) { this->normalSamplingRate = rate; this->normalSamplingLevel = this->getSamplingMapIndex(rate); }

  float getSamplingRate() const { return samplingRate; }
  unsigned int getSamplingLevel() const { return samplingLevel; }
  float getSize() const { return histogramSize; }
  float getNormalSamplingRate() const { return normalSamplingRate; }
  unsigned int getNormalSamplingLevel() const { return normalSamplingLevel; }
  float getNormalScale() const { return normalScale; }

  float getOctreeMinimumVolumeSize() const { return octreeMinimumVolumeSize; }
  float getOctreeExpansion() const { return octreeExpansion; }
  float getOctreeResolutionThreshold() const { return octreeResolutionThreshold; }

  //! returns the index of the nearest sampling resolution to a given resolution
  int getSamplingMapIndex(float resolution) const;

  void setEntropyCalculationMode(EntropyCalculationMode mode) { this->entropyMode = mode; }
  void setCrossProducteWeightMethod(CrossProductWeightMethod method) { this->cpWeightMethod = method; }

  void setFeatureInfluenceRadius(float radius) { this->featureInfluenceRadius = radius; }
  void setMinimumCornerness(float cornerness) { this->minimumCornerness3D = cornerness; }
  void setMinimumEntropy(float entropy) { this->minimumEntropy = entropy; }

  void setOctreeMinimumVolumeSize(float size)
  {
    this->octreeMinimumVolumeSize = size;
    this->setSamplingRate(this->samplingRate);
    this->setNormalSamplingRate(this->normalSamplingRate);
  }
  void setOctreeExpansion(float expansion)
  {
    this->octreeExpansion = expansion;
    this->setSamplingRate(this->samplingRate);
    this->setNormalSamplingRate(this->normalSamplingRate);
  }
  void setOctreeResolutionThreshold(float threshold) { this->octreeResolutionThreshold = threshold; }

  void setAdditionalPointsOnDepthBorders(bool addPoints) { this->additionalPointsOnDepthBorders = addPoints; }
  void setIgnoreBackgroundDetections(bool ignore) { this->ignoreBackgroundDetections = ignore; }
  void setImprovedLocalization(bool localize) { this->improvedLocalization = localize; }
  void setLimitOctreeResolution(bool limit) { this->limitOctreeResolution = limit; }

protected:

  float samplingRate;
  unsigned int samplingLevel;

  float histogramSize;
  float histogramRadius;

  float normalScale;
  float normalScaleRadius;

  float normalSamplingRate;
  unsigned int normalSamplingLevel;

  float featureInfluenceRadius;
  float minimumEntropy;
  float minimumCornerness3D;
  float curvatureRadius;

  EntropyCalculationMode entropyMode;

  float octreeMinimumVolumeSize;
  float octreeExpansion;
  float octreeResolutionThreshold;

  bool additionalPointsOnDepthBorders;
  bool ignoreBackgroundDetections;
  bool improvedLocalization;
  bool limitOctreeResolution;

  CrossProductWeightMethod cpWeightMethod;

  template <typename PointT>
  friend class SURE_Estimator;

  friend std::ostream& operator<<(std::ostream& stream, const sure::Configuration& config);

};

std::ostream& operator<<(std::ostream& stream, const sure::Configuration& config);

}


#endif /* CONFIGURATION_H_ */
