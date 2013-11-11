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

#include <sure/feature/feature.h>


void sure::feature::Feature::resetDescriptor(unsigned distanceClasses)
{
  descriptors_.clear();
  descriptors_.resize(distanceClasses);
  distanceClassSize_ = radius_ / (Scalar) descriptors_.size();
}

void sure::feature::Feature::normalizeDescriptor()
{
  for(unsigned int i=0; i<descriptors_.size(); ++i)
  {
    descriptors_[i].normalize();
  }
}

sure::Scalar sure::feature::Feature::distanceTo(const Feature& rhs, Scalar shapeWeight, Scalar colorWeight, Scalar lightnessWeight) const
{
  if( this->descriptors_.size() != rhs.descriptors_.size() || fabs(this->radius_ - rhs.radius_) > 1e-3 )
  {
    return std::numeric_limits<Scalar>::infinity();
  }
  Scalar distance(0.0);
  for(unsigned int i=0; i<descriptors_.size(); ++i)
  {
    distance += descriptors_[i].distanceTo(rhs.descriptors_[i], shapeWeight, colorWeight, lightnessWeight);
  }
  return (distance / (Scalar) descriptors_.size());
}

int sure::feature::Feature::createDescriptor(const NodeVector& nodes, Scalar referenceLightness, unsigned distanceClasses)
{
  int usedPoints(std::numeric_limits<int>::max());
  resetDescriptor(distanceClasses);
  usedPoints = std::min(usedPoints, createShapedescriptor(nodes));
  usedPoints = std::min(usedPoints, createColordescriptor(nodes));
  usedPoints = std::min(usedPoints, createLightnessdescriptor(nodes, referenceLightness));
  normalizeDescriptor();
  hasDescriptor_ = true;
  return usedPoints;
}

int sure::feature::Feature::createShapedescriptor(const NodeVector& nodes)
{
  int usedPoints(0);
  for(unsigned i=0; i<nodes.size(); ++i)
  {
    Node* currNode = nodes[i];
    NormalPayload* payload = currNode->opt() ? static_cast<NormalPayload*>(currNode->opt()) : NULL;
    if( !payload || !payload->normal_.isStable() )
    {
      continue;
    }
    HistoType alpha, phi, theta;
    sure::descriptor::calculateSurfletPairRelations(position_, normal_.vector(), currNode->fixed().getMeanPosition(), payload->normal_.vector(), alpha, phi, theta);
    if( std::isfinite(alpha) && std::isfinite(phi) && std::isfinite(theta) )
    {
      unsigned dClass = getDistanceClass(position_, currNode->fixed().getMeanPosition());
      descriptors_.at(dClass).shape().insertValues(alpha, phi, theta);
      usedPoints++;
    }
  }
  return usedPoints;
}

int sure::feature::Feature::createColordescriptor(const NodeVector& nodes)
{
  int usedPoints(0);
  for(unsigned i=0; i<nodes.size(); ++i)
  {
    Node* currNode = nodes[i];
    Scalar hue, saturation, lightness;
    sure::descriptor::convertRGBtoHSL(currNode->fixed().getMeanColor()[0], currNode->fixed().getMeanColor()[1], currNode->fixed().getMeanColor()[2], hue, saturation, lightness);
    if( std::isfinite(hue) && std::isfinite(saturation) )
    {
      unsigned dClass = getDistanceClass(position_, currNode->fixed().getMeanPosition());
      descriptors_.at(dClass).color().insertValue(hue, saturation);
      usedPoints++;
    }
  }
  return usedPoints;
}

int sure::feature::Feature::createLightnessdescriptor(const NodeVector& nodes, Scalar referenceLightness)
{
  for(unsigned i=0; i<descriptors_.size(); ++i)
  {
    descriptors_[i].lightness().setLightness(referenceLightness);
  }

  int usedPoints(0);
  for(unsigned i=0; i<nodes.size(); ++i)
  {
    Node* currNode = nodes[i];
    Scalar hue, saturation, lightness;
    sure::descriptor::convertRGBtoHSL(currNode->fixed().getMeanColor()[0], currNode->fixed().getMeanColor()[1], currNode->fixed().getMeanColor()[2], hue, saturation, lightness);
    if( std::isfinite(hue) && std::isfinite(saturation) )
    {
      unsigned dClass = getDistanceClass(position_, currNode->fixed().getMeanPosition());
      descriptors_.at(dClass).lightness().insertValue(lightness);
      usedPoints++;
    }
  }
  return usedPoints;
}

unsigned sure::feature::Feature::getDistanceClass(const Vector3& center, const Vector3& pos) const
{
  int dClass = 0;
  if( this->descriptors_.size() > 1 )
  {
    Vector3 distance = (center - pos);
    dClass = std::max(dClass, (int) floor(distance[0] / distanceClassSize_));
    dClass = std::max(dClass, (int) floor(distance[1] / distanceClassSize_));
    dClass = std::max(dClass, (int) floor(distance[2] / distanceClassSize_));
  }
  return (unsigned) std::min(dClass, (int) descriptors_.size()-1);
}

std::ostream& sure::feature::operator<<(std::ostream& stream, const Feature& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(2);
  stream << "Feature - radius: " << rhs.radius_ << " - descriptor splits: " << rhs.descriptors_.size() << " - distance split: " << rhs.distanceClassSize_ << "\n";
  stream << "Position: " << rhs.position_[0] << "/" << rhs.position_[1] << "/" << rhs.position_[2] << " - normal: " << rhs.normal_;
  for(unsigned i=0; i<rhs.descriptors_.size(); ++i)
  {
    stream << "Descriptor " << i+1 << ": " << rhs.descriptors_[i];
  }
  stream.unsetf(std::ios_base::fixed);
  return stream;
}
