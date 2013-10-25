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

#include "sure/feature.h"

sure::Feature sure::Feature::operator +(const sure::Feature& rhs) const
{
  sure::Feature lhs(*this);
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    lhs.pfDescriptor[i] += rhs.pfDescriptor[i];
    lhs.colorDescriptor[i] += rhs.colorDescriptor[i];
    lhs.lightnessDescriptor[i] += rhs.lightnessDescriptor[i];
  }
  return lhs;
}

sure::Feature& sure::Feature::operator +=(const sure::Feature& rhs)
{
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    this->pfDescriptor[i] += rhs.pfDescriptor[i];
    this->colorDescriptor[i] += rhs.colorDescriptor[i];
    this->lightnessDescriptor[i] += rhs.lightnessDescriptor[i];
  }
  return *this;
}

sure::Feature sure::Feature::operator *(const double rhs) const
{
  sure::Feature lhs(*this);
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    lhs.pfDescriptor[i] *= rhs;
    lhs.colorDescriptor[i] *= rhs;
    lhs.lightnessDescriptor[i] *= rhs;
  }
  return lhs;
}

sure::Feature& sure::Feature::operator *=(const double rhs)
{
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    this->pfDescriptor[i] *= rhs;
    this->colorDescriptor[i] *= rhs;
    this->lightnessDescriptor[i] *= rhs;
  }
  return *this;
}

sure::Feature sure::Feature::operator /(const double rhs) const
{
  sure::Feature lhs(*this);
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    lhs.pfDescriptor[i] /= rhs;
    lhs.colorDescriptor[i] /= rhs;
    lhs.lightnessDescriptor[i] /= rhs;
  }
  return lhs;
}

sure::Feature& sure::Feature::operator /=(const double rhs)
{
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    this->pfDescriptor[i] /= rhs;
    this->colorDescriptor[i] /= rhs;
    this->lightnessDescriptor[i] /= rhs;
  }
  return *this;
}

void sure::Feature::reset()
{
  sure::ColorSurflet::reset();
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    pfDescriptor[i].clear();
    colorDescriptor[i].clear();
    lightnessDescriptor[i].clear();
  }
}

void sure::Feature::print() const
{
  sure::ColorSurflet::print();
  std::cout << std::fixed << std::setprecision(3) << "[Feature] Entropy: " << mEntropy << " Radius: " << mRadius << " PointCloud Index: " << mPointCloudIndex << std::endl;
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    pfDescriptor[i].print();
    colorDescriptor[i].print();
    lightnessDescriptor[i].print();
  }
  std::cout << "\n";
}

bool sure::Feature::hasNormalizedDescriptors() const
{
  bool normalized = true;
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    normalized = normalized && pfDescriptor[i].isNormalized() && colorDescriptor[i].isNormalized() && lightnessDescriptor[i].isNormalized();
  }
  return normalized;
}

double sure::Feature::distanceTo(const sure::Feature& rhs, double shapeWeight, double colorWeight, double lightnessWeight) const
{
  if( this->hasNormalizedDescriptors() && rhs.hasNormalizedDescriptors() && this->numberOfDescriptors == rhs.numberOfDescriptors )
  {
    double distance = 0.0;
    for(unsigned int i=0; i<numberOfDescriptors; ++i)
    {
      distance += pfDescriptor[i].distanceTo(rhs.pfDescriptor[i]) * shapeWeight;
      distance += colorDescriptor[i].distanceTo(rhs.colorDescriptor[i]) * colorWeight;
      distance += lightnessDescriptor[i].distanceTo(rhs.lightnessDescriptor[i]) * lightnessWeight;
    }
    return distance / ((shapeWeight+colorWeight+lightnessWeight) * (double) numberOfDescriptors);
  }
  return INFINITY;
}

double sure::Feature::distanceTo(const sure::Feature& rhs) const
{
  return this->distanceTo(rhs, 1.0, 1.0, 1.0);
}

int sure::Feature::determineDistanceClass(const sure::Surflet& surflet)
{
  int dClass = 0;
  if( this->numberOfDescriptors > 1 )
  {
    float distance = fabsf(surflet.position()[0] - this->position()[0]) + fabsf(surflet.position()[1] - this->position()[1]) + fabsf(surflet.position()[2] - this->position()[2]);
    float classSize = (3.f * this->mRadius) / float(this->numberOfDescriptors);
    dClass = floor(distance / classSize);
  }
  return dClass;
}

void sure::Feature::calculateDescriptor(const std::vector<sure::ColorSurflet>& surflets, bool normalize)
{
  double alpha = 0.0, phi = 0.0, theta = 0.0;
  double hue = 0.0, saturation = 0.0, lightness = 0.0;
  unsigned int distanceClass = 0;

  sure::convertRGBtoHSL(mRed, mGreen, mBlue, hue, saturation, lightness);
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    lightnessDescriptor[i].setLightness(lightness);
  }

  for(unsigned int i=0; i<surflets.size(); ++i)
  {
    sure::convertRGBtoHSL(surflets[i].r(), surflets[i].g(), surflets[i].b(), hue, saturation, lightness);
    sure::calculateSurfletPairRelations(*this, surflets[i], alpha, phi, theta);
    distanceClass = determineDistanceClass(surflets[i]);

    if( isinf(alpha) || isinf(phi) || isinf(theta) || isnan(alpha) || isnan(phi) || isnan(theta) || distanceClass >= numberOfDescriptors )
    {
      continue;
    }

    pfDescriptor[distanceClass].insertValues(alpha, phi, theta);
    colorDescriptor[distanceClass].insertValue(hue, saturation);
    lightnessDescriptor[distanceClass].insertValue(lightness);
  }
  if( normalize )
  {
    for(unsigned int i=0; i<numberOfDescriptors; ++i)
    {
      pfDescriptor[i].normalize();
      colorDescriptor[i].normalize();
      lightnessDescriptor[i].normalize();
    }
  }
}

void sure::Feature::calculateDescriptor(const std::vector<OctreeNode* >& nodes, bool normalize)
{
  double alpha = 0.0, phi = 0.0, theta = 0.0;
  double hue = 0.0, saturation = 0.0, lightness = 0.0;
  unsigned int distanceClass = 0;

  sure::convertRGBtoHSL(mRed, mGreen, mBlue, hue, saturation, lightness);
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    lightnessDescriptor[i].setLightness(lightness);
  }

  for(unsigned int i=0; i<nodes.size(); ++i)
  {
    if( nodes[i]->value.statusOfNormal != sure::OctreeValue::NORMAL_STABLE )
    {
      continue;
    }
    sure::convertRGBtoHSL(nodes[i]->value.r(), nodes[i]->value.g(), nodes[i]->value.b(), hue, saturation, lightness);

    sure::Surflet surflet;
    surflet.setPosition(nodes[i]->closestPosition.p[0], nodes[i]->closestPosition.p[1], nodes[i]->closestPosition.p[2]);
    surflet.setNormal(nodes[i]->value.normal[0], nodes[i]->value.normal[1], nodes[i]->value.normal[2]);

    sure::calculateSurfletPairRelations(*this, surflet, alpha, phi, theta);
    distanceClass = determineDistanceClass(surflet);

    if( isinf(alpha) || isinf(phi) || isinf(theta) || isnan(alpha) || isnan(phi) || isnan(theta) || distanceClass >= numberOfDescriptors )
    {
      continue;
    }

    pfDescriptor[distanceClass].insertValues(alpha, phi, theta);
    colorDescriptor[distanceClass].insertValue(hue, saturation);
    lightnessDescriptor[distanceClass].insertValue(lightness);
  }
  if( normalize )
  {
    for(unsigned int i=0; i<numberOfDescriptors; ++i)
    {
      pfDescriptor[i].normalize();
      colorDescriptor[i].normalize();
      lightnessDescriptor[i].normalize();
    }
  }
//  ROS_INFO("Descriptorcreation: %i/%i Normals bad.", nonormals, (int) nodes.size());
}

void sure::Feature::createRandomDescriptor()
{
  for(unsigned int i=0; i<numberOfDescriptors; ++i)
  {
    pfDescriptor[i].fillRandom();
    colorDescriptor[i].fillRandom();
    lightnessDescriptor[i].fillRandom();
  }
}

//std::ofstream& sure::operator<<(std::ofstream& stream, const sure::Feature& rhs)
//{
//  stream << (sure::ColorSurflet) rhs;
//  stream << std::fixed << std::setprecision(3) << "[Feature] Entropy: " << rhs.entropy << " Radius: " << rhs.radius << " Cornerness: " << rhs.cornerness3D << std::endl;
//  for(unsigned int i=0; i<rhs.descriptorCount; ++i)
//  {
////    stream << rhs.pfDescriptor[i];
////    stream << rhs.colorDescriptor[i];
////    stream << rhs.lightnessDescriptor[i];
//  }
//  return stream;
//}

BOOST_CLASS_VERSION(sure::Feature, 0)
