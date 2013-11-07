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

#include <sure/data/configuration.h>

std::ostream& sure::operator<<(std::ostream& stream, const sure::Configuration& config)
{
  stream << "#\n" << "# SURE 3D Configuration\n" << "#\n";
  stream << "# Scales: ";
  for(unsigned int i=0; i<config.Scales.size(); ++i)
  {
    stream << floor(config.Scales.at(i)*100.f + 0.5f) << "cm";
    if( i < config.Scales.size() - 1 )
    {
      stream << ", ";
    }
    else
    {
      stream << "\n";
    }
  }

  stream << "# Samplingrate: " << floor(config.Samplingrate*100.f + 0.5f) << "cm - normal samplingrate: " << floor(config.NormalSamplingrate*100.f + 0.5f) << "cm - normal region size: " << floor(config.NormalRegionSize*100.f + 0.5f) << "cm - normal histogram influence: " << (config.NormalInfluenceRadius*180.0)/M_PI << "deg \n";
  std::cout << "# Descriptor samplingrate: " << floor(config.DescriptorSamplingrate*100.f + 0.5f) << "cm - number of descriptor distance classes: " << config.DescriptorNumberOfDistanceClasses << "\n";

  stream << "# Minimum entropy threshold: " << config.MinimumEntropyThreshold << " - minimum cornerness threshold: " << config.MinimumCornernessThreshold;
  if( config.DistanceThreshold > 0.f )
  {
    stream << " - maximum allowed distance from sensor: " << config.DistanceThreshold << "m\n";
  }
  else
  {
    stream << "\n";
  }

  stream << "# Improve Feature Localization: " << config.ImproveLocalization << " - Additional Points on Depth Borders: " << config.AdditionalPointsOnDepthBorders << " - Ignore Background Detections: " << config.IgnoreBackgroundDetections << "\n";
  stream << "# Entropy Calculation: ";
  switch(config.EntropyMode)
  {
    default:
    case NORMALS:
      stream << " Normals\n";
      break;
    case CROSS_PRODUCTS_PAIRWISE:
      stream << " Pairwise cross-products between normals\n";
      break;
    case CROSS_PRODUCTS_W_MAIN:
      stream << " Cross-products between main normal an neighboring normals\n";
      break;
  }
  return stream;
}

BOOST_CLASS_VERSION(sure::Configuration, 8)
