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

#ifndef HISTOGRAM_EMD_H_
#define HISTOGRAM_EMD_H_

#include "sure/histogram_base.h"
#include <FastEMD/emd_hat.hpp>

namespace sure
{

template<unsigned int histoSize>
class DescriptorHistogramWithEMDistance : public sure::HistogramBase
{
public:

  static const double MAX_DISTANCE;
  static const std::string HISTOGRAM_NAME;

  DescriptorHistogramWithEMDistance(double min, double max, const std::string& name = sure::DescriptorHistogramWithEMDistance<histoSize>::HISTOGRAM_NAME) : sure::HistogramBase(histoSize, min, max, name)
  {
    clear();
  }

  DescriptorHistogramWithEMDistance(const sure::DescriptorHistogramWithEMDistance<histoSize>& obj)
    : sure::HistogramBase(histoSize, obj.minimum, obj.maximum, obj.name)
  {
    this->operator=(obj);
  }
  virtual ~DescriptorHistogramWithEMDistance() {}

  sure::DescriptorHistogramWithEMDistance<histoSize>& operator=(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs);
  sure::DescriptorHistogramWithEMDistance<histoSize>& operator=(const sure::HistogramBase& rhs);
  sure::DescriptorHistogramWithEMDistance<histoSize> operator +(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs) const;
  sure::DescriptorHistogramWithEMDistance<histoSize>& operator +=(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs);
  sure::DescriptorHistogramWithEMDistance<histoSize> operator *(const double rhs) const;
  sure::DescriptorHistogramWithEMDistance<histoSize>& operator *=(const double rhs);
  sure::DescriptorHistogramWithEMDistance<histoSize> operator /(const double rhs) const;
  sure::DescriptorHistogramWithEMDistance<histoSize>& operator /=(const double rhs);

  virtual void clear();
  virtual void normalize();
  virtual void print() const;
  virtual void printMatrix() const { print(this->DISTANCE_MATRIX); }
  void print(const std::vector<std::vector<double> >& distMatrix) const;

  virtual void insertValue(double value);
  double distanceTo(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs) const;

  void fillRandom();

protected:

  std::vector<double> histogram;

  static const std::vector<std::vector<double> > DISTANCE_MATRIX;

  template<unsigned int size>
  friend std::ostream& operator<<(std::ostream& stream, const sure::DescriptorHistogramWithEMDistance<size>& rhs);

private:

  DescriptorHistogramWithEMDistance() : sure::HistogramBase(0, 0.0, 0.0, "") {}

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<sure::HistogramBase>(*this);
    for(unsigned int i=0; i<histogram.size(); ++i)
    {
      ar & histogram.at(i);
    }
  }

};

template<unsigned int histoSize>
std::vector<std::vector<double> > initEMDMatrix();

}

#include "sure/impl/histogram_emd.hpp"

#endif /* HISTOGRAM_EMD_H_ */
