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

#ifndef POINT_FEATURE_DESCRIPTOR_H_
#define POINT_FEATURE_DESCRIPTOR_H_

#include "sure/histogram_l2.h"

namespace sure
{

const double ALPHA_MIN = -1.0;
const double ALPHA_MAX = 1.0;
const double PHI_MIN = -1.0;
const double PHI_MAX = 1.0;
const double THETA_MIN = -M_PI;
const double THETA_MAX = M_PI;

class PointFeatureDescriptor
{
public:

  PointFeatureDescriptor(int histoSize = 11)
  : alphaHistogram(ALPHA_MIN, ALPHA_MAX, histoSize),
    phiHistogram(PHI_MIN, PHI_MAX, histoSize),
    thetaHistogram(THETA_MIN, THETA_MAX, histoSize) {}

  sure::PointFeatureDescriptor operator +(const sure::PointFeatureDescriptor& rhs) const;
  sure::PointFeatureDescriptor& operator +=(const sure::PointFeatureDescriptor& rhs);
  sure::PointFeatureDescriptor operator *(const double rhs) const;
  sure::PointFeatureDescriptor& operator *=(const double rhs);
  sure::PointFeatureDescriptor operator /(const double rhs) const;
  sure::PointFeatureDescriptor& operator /=(const double rhs);

  void clear();
  void normalize();
  void print() const;

  bool isNormalized() const { return alphaHistogram.isNormalized() && phiHistogram.isNormalized() && thetaHistogram.isNormalized(); }
//  virtual bool isValid() const { return valid; }
//  pcl::InterestPoint getInterestPointFromFeature() const;

  void insertValues(double alpha, double phi, double theta);
  double distanceTo(const sure::PointFeatureDescriptor& rhs) const;

  void fillRandom();

protected:

  sure::DescriptorHistogramWithL2Distance alphaHistogram; // Cosine from polar angle of normals
  sure::DescriptorHistogramWithL2Distance phiHistogram; // direction between surflet points
  sure::DescriptorHistogramWithL2Distance thetaHistogram; // Azimuth Angle between normals

};

}

#endif /* POINT_FEATURE_DESCRIPTOR_H_ */
