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

#include "sure/point_feature_descriptor.h"

sure::PointFeatureDescriptor sure::PointFeatureDescriptor::operator +(const sure::PointFeatureDescriptor& rhs) const
{
  sure::PointFeatureDescriptor lhs(*this);
  lhs.alphaHistogram += rhs.alphaHistogram;
  lhs.phiHistogram += rhs.phiHistogram;
  lhs.thetaHistogram += rhs.phiHistogram;
  return lhs;
}

sure::PointFeatureDescriptor& sure::PointFeatureDescriptor::operator +=(const sure::PointFeatureDescriptor& rhs)
{
  this->alphaHistogram += rhs.alphaHistogram;
  this->phiHistogram += rhs.phiHistogram;
  this->thetaHistogram += rhs.phiHistogram;
  return *this;
}

sure::PointFeatureDescriptor sure::PointFeatureDescriptor::operator *(const double rhs) const
{
  sure::PointFeatureDescriptor lhs(*this);
  lhs.alphaHistogram *= rhs;
  lhs.phiHistogram *= rhs;
  lhs.thetaHistogram *= rhs;
  return lhs;
}

sure::PointFeatureDescriptor& sure::PointFeatureDescriptor::operator *=(const double rhs)
{
  this->alphaHistogram *= rhs;
  this->phiHistogram *= rhs;
  this->thetaHistogram *= rhs;
  return *this;
}

sure::PointFeatureDescriptor sure::PointFeatureDescriptor::operator /(const double rhs) const
{
  sure::PointFeatureDescriptor lhs(*this);
  lhs.alphaHistogram /= rhs;
  lhs.phiHistogram /= rhs;
  lhs.thetaHistogram /= rhs;
  return lhs;
}

sure::PointFeatureDescriptor& sure::PointFeatureDescriptor::operator /=(const double rhs)
{
  this->alphaHistogram /= rhs;
  this->phiHistogram /= rhs;
  this->thetaHistogram /= rhs;
  return *this;
}


void sure::PointFeatureDescriptor::clear()
{
  alphaHistogram.clear();
  phiHistogram.clear();
  thetaHistogram.clear();
}

void sure::PointFeatureDescriptor::normalize()
{
  alphaHistogram.normalize();
  phiHistogram.normalize();
  thetaHistogram.normalize();
}

void sure::PointFeatureDescriptor::print() const
{
  alphaHistogram.print();
  phiHistogram.print();
  thetaHistogram.print();
}

void sure::PointFeatureDescriptor::insertValues(double alpha, double phi, double theta)
{
  this->alphaHistogram.insertValue(alpha);
  this->phiHistogram.insertValue(phi);
  this->thetaHistogram.insertValue(theta);
}

double sure::PointFeatureDescriptor::distanceTo(const sure::PointFeatureDescriptor& rhs) const
{
  if( this->isNormalized() && rhs.isNormalized() )
  {
    double distance = 0.0;
    distance += this->alphaHistogram.distanceTo(rhs.alphaHistogram);
    distance += this->phiHistogram.distanceTo(rhs.phiHistogram);
    distance += this->thetaHistogram.distanceTo(rhs.thetaHistogram);
    return (distance / 3.0);
  }
  return INFINITY;
}

void sure::PointFeatureDescriptor::fillRandom()
{
  alphaHistogram.fillRandom();
  phiHistogram.fillRandom();
  thetaHistogram.fillRandom();
}

BOOST_CLASS_VERSION(sure::PointFeatureDescriptor, 0)
