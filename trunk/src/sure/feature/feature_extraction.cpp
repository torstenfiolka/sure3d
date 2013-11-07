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

#include <sure/feature/feature_extraction.h>

sure::feature::Feature sure::feature::createFeature(const Octree& octree, const Vector3& position, Scalar samplingrate, Scalar radius, const sure::normal::Normal& normal, unsigned distanceClasses)
{
  Feature feature;
  feature.position() = position;
  feature.radius() = radius;
  feature.normal() = normal;
  createDescriptor(octree, feature, samplingrate, distanceClasses);
  return feature;
}

sure::feature::Feature sure::feature::createFeature(const Octree& octree, const Vector3& position, Scalar samplingrate, Scalar radius, const Vector3& viewPoint, unsigned distanceClasses)
{
  Feature feature;
  feature.position() = position;
  feature.radius() = radius;
  if( sure::normal::estimateNormal(octree, position, radius, feature.normal()) )
  {
    sure::normal::orientateNormal(position, feature.normal(), viewPoint);
    createDescriptor(octree, feature, samplingrate, distanceClasses);
  }
  return feature;
}

unsigned sure::feature::createDescriptors(const Octree& octree, std::vector<Feature>& features, Scalar samplingrate, const Vector3& viewPoint, unsigned distanceClasses)
{
  unsigned sum(0);
  for(unsigned featureIndex=0; featureIndex<features.size(); ++featureIndex)
  {
    Feature& currFeature = features[featureIndex];

    if( sure::normal::estimateNormal(octree, currFeature.position(), currFeature.radius(), currFeature.normal()) )
    {
      sure::normal::orientateNormal(currFeature.position(), currFeature.normal(), viewPoint);
      if( createDescriptor(octree, currFeature, samplingrate, distanceClasses) )
      {
        sum++;
      }
    }
  }
  return sum;
}

bool sure::feature::createDescriptor(const Octree& octree, Feature& feature, Scalar samplingrate, unsigned distanceClasses)
{
  Scalar hue, saturation, lightness;
  sure::payload::PointsRGB regionIntegrate;
  octree.integratePayload(feature.position(), feature.radius(), regionIntegrate);

  sure::descriptor::convertRGBtoHSL(regionIntegrate.red(), regionIntegrate.green(), regionIntegrate.blue(), hue, saturation, lightness);

  NodeVector vec = octree.getNodes(feature.position(), feature.radius(), samplingrate);
  unsigned usedPoints(0), usedColors(0), usedLightness(0);

  feature.createDescriptor(vec, lightness, distanceClasses);

  return feature.hasDescriptor();
}
