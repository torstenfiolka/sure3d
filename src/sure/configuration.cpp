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

#include <sure/configuration.h>

void sure::Configuration::reset()
{
  samplingRate = 0.04f;
  histogramSize = samplingRate * 3.f;
  normalSamplingRate = samplingRate * 0.5f;

  samplingLevel = getSamplingMapIndex(samplingRate);

  histogramRadius = histogramSize * 0.5f;

  normalSamplingLevel = getSamplingMapIndex(normalSamplingRate);
  normalScale = samplingRate;
  normalScaleRadius = normalScale * 0.5f;
  curvatureRadius = normalSamplingRate*1.5f;

  featureInfluenceRadius = histogramRadius * 2.f;

  minimumEntropy = 0.6f;
  minimumCornerness3D = 0.15f;

  entropyMode = sure::NORMALS;
  cpWeightMethod = sure::INVERSE_ABSOLUTE_DOT_PRODUCT;

  additionalPointsOnDepthBorders = false;
  ignoreBackgroundDetections = false;
  improvedLocalization = true;
  limitOctreeResolution = true;

  octreeMinimumVolumeSize = 0.01f;
  octreeExpansion = 40.96f;
  octreeResolutionThreshold = 0.004;
}

int sure::Configuration::getSamplingMapIndex(float resolution) const
{
  float size = octreeExpansion, minDistance = INFINITY;
  int level = 0, bestLevel = 0;
  while( size > octreeMinimumVolumeSize )
  {
    if( fabs(size-resolution) < minDistance)
    {
      minDistance = fabs(size-resolution);
      bestLevel = level;
    }
    level++;
    size = size * 0.5;
  }
  return bestLevel;
}


std::ostream& sure::operator<<(std::ostream& stream, const sure::Configuration& config)
{
  stream << "#\n" << "# Configuration\n" << "#\n";
  stream << "# Samplingrate: " << config.samplingRate << " Corresponding octree Level: " << config.samplingLevel << " Histogram Size (scale): " << config.histogramSize << "\n";
  stream << "# Normal Samplingrate: " << config.normalSamplingRate << " Ccorresponding octree level: " << config.normalSamplingLevel << " Normal Scale: " << config.normalScale << std::endl;
  stream << "# Feature influence radius: " << config.featureInfluenceRadius << " Minimum entropy: " << config.minimumEntropy << " Minimum cornerness: " << config.minimumCornerness3D << std::endl;
  stream << "# Improved feature localization: " << config.improvedLocalization << " Additional points on depth borders: " << config.additionalPointsOnDepthBorders << " Ignore background detections: " << config.ignoreBackgroundDetections << std::endl;
  stream << "# entropy calculation: " << (int) config.entropyMode << " histogram weighting: " << config.cpWeightMethod << std::endl;
  stream << "# Octree Expansion: " << config.octreeExpansion << " Minimum Volume Size: " << config.octreeMinimumVolumeSize << " (Level " << config.getSamplingMapIndex(config.octreeMinimumVolumeSize) << ") Limited resolution: " << config.limitOctreeResolution << " (" << config.octreeResolutionThreshold << ")\n#\n";
  return stream;
}
