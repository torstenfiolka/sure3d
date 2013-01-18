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

#ifndef HISTOGRAM_L2_H_
#define HISTOGRAM_L2_H_

#include "sure/histogram_base.h"

namespace sure
{


class DescriptorHistogramWithL2Distance : public sure::HistogramBase
{
public:

  // Square root of 2
  static const double MAX_DISTANCE = 1.414213562;
  static const std::string HISTOGRAM_NAME;

  DescriptorHistogramWithL2Distance(double min, double max, unsigned int histoSize = 11, const std::string& name = sure::DescriptorHistogramWithL2Distance::HISTOGRAM_NAME);
  DescriptorHistogramWithL2Distance(const sure::DescriptorHistogramWithL2Distance& obj) : sure::HistogramBase(obj.histogramSize, obj.minimum, obj.maximum, obj.name) , histogram(NULL)
  {
    this->operator =(obj);
  }
  DescriptorHistogramWithL2Distance(const sure::HistogramBase& obj);
  virtual ~DescriptorHistogramWithL2Distance();

  sure::DescriptorHistogramWithL2Distance& operator=(const sure::DescriptorHistogramWithL2Distance& rhs);
  sure::DescriptorHistogramWithL2Distance& operator=(const sure::HistogramBase& rhs);
  sure::DescriptorHistogramWithL2Distance operator +(const sure::DescriptorHistogramWithL2Distance& rhs) const;
  sure::DescriptorHistogramWithL2Distance& operator +=(const sure::DescriptorHistogramWithL2Distance& rhs);
  sure::DescriptorHistogramWithL2Distance operator *(const double rhs) const;
  sure::DescriptorHistogramWithL2Distance& operator *=(const double rhs);
  sure::DescriptorHistogramWithL2Distance operator /(const double rhs) const;
  sure::DescriptorHistogramWithL2Distance& operator /=(const double rhs);

  void clear();
  void normalize();
  void print() const;

  void insertValue(double value);
  double distanceTo(const sure::DescriptorHistogramWithL2Distance& rhs) const;

  void fillRandom();

protected:

  double *histogram;

};

}

#endif /* HISTOGRAM_L2_H_ */
