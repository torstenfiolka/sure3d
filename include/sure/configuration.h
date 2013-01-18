// Software License Agreement (BSD License)
//
// Copyright (c) 2012, Fraunhofer FKIE/US
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

const float OCTREE_MINIMUM_VOLUME_SIZE = 0.01f;
const float OCTREE_INITIAL_SIZE = 81.92f;

//! returns the index of the nearest sampling resolution to a given resolution
int getSamplingMapIndex(float resolution);

//! stores configuration data
class Configuration
{
public:

  enum EntropyCalculationMode
  {
    NORMALS = 0,
    CROSSPRODUCTS_ALL_NORMALS_WITH_MAIN_NORMAL,
    CROSSPRODUCTS_ALL_NORMALS_PAIRWISE
  };

  Configuration()
  {
    reset();
  }

  void reset();

  float getSamplingRate() const { return samplingRate; }
  float getSize() const { return histogramSize; }
  float getNormalSamplingRate() const { return normalSamplingRate; }
  float getNormalScale() const { return normalScale; }

  void setSamplingRate(float rate) { this->samplingRate = rate; this->samplingLevel = sure::getSamplingMapIndex(rate); }
  void setSize(float size) { this->histogramSize = size; this->histogramRadius = size*0.5f; this->featureInfluenceRadius = size; }
  void setNormalsScale(float scale) { this->normalScale = scale; this->normalScaleRadius = scale*0.5f; }
  void setNormalSamplingRate(float rate) { this->normalSamplingRate = rate; this->normalSamplingLevel = sure::getSamplingMapIndex(rate); }

  void setEntropyCalculationMode(EntropyCalculationMode mode) { this->entropyMode = mode; }
  void setCrossProducteWeightMethod(int method) { this->histogramWeightMethod = method; }

  void setFeatureInfluenceRadius(float radius) { this->featureInfluenceRadius = radius; }
  void setMinimumCornerness(float cornerness) { this->minimumCornerness3D = cornerness; }
  void setMinimumEntropy(float entropy) { this->minimumEntropy = entropy; }
//  void setMinimumOctreeVolumeSize(float size) { this->minimumOctreeVolumeSize = size; }

  void setAdditionalPointsOnDepthBorders(bool addPoints) { this->additionalPointsOnDepthBorders = addPoints; }
  void setIgnoreBackgroundDetections(bool ignore) { this->ignoreBackgroundDetections = ignore; }
  void setImprovedLocalization(bool localize) { this->improvedLocalization = localize; }
  void setLimitOctreeResolution(bool limit) { this->limitOctreeResolution = limit; }
  void setMultiResolutionNormals(bool multiResolution) { this->multiResolutionNormals = multiResolution; }

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

  EntropyCalculationMode entropyMode;

  float minimumOctreeVolumeSize;

  bool additionalPointsOnDepthBorders;
  bool ignoreBackgroundDetections;
  bool improvedLocalization;
  bool limitOctreeResolution;
  bool multiResolutionNormals;

  int histogramWeightMethod;

  template <typename PointT>
  friend class SURE_Estimator;

  friend std::ostream& operator<<(std::ostream& stream, const sure::Configuration& config);

};

std::ostream& operator<<(std::ostream& stream, const sure::Configuration& config);

}


#endif /* CONFIGURATION_H_ */
