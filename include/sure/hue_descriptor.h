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

#ifndef HUE_DESCRIPTOR_H_
#define HUE_DESCRIPTOR_H_

#include "sure/histogram_emd.h"

namespace sure
{

static const int COLOR_HISTOGRAM_SIZE = 25;

std::vector<std::vector<double> > initHueEMDMatrix();

class HueDescriptor : public sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>
{
public:

  static const double MIN_HUE = 0.0;
  static const double MAX_HUE = 6.0;
  static const double MAX_SATURATION_DISTANCE;
  static const std::string HISTOGRAM_NAME;

  HueDescriptor()
    : sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>(MIN_HUE, MAX_HUE, sure::HueDescriptor::HISTOGRAM_NAME)
  {
    sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>::binSize =  sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>::range / (double) (COLOR_HISTOGRAM_SIZE - 1);
  }

  HueDescriptor(const sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>& obj)
    : sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>(obj)
  {
    sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>::binSize =  sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>::range / (double) (COLOR_HISTOGRAM_SIZE - 1);
  }

  virtual ~HueDescriptor() { }

  sure::HueDescriptor& operator=(const sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>& obj);

  virtual void printMatrix() const
  {
    sure::DescriptorHistogramWithEMDistance<COLOR_HISTOGRAM_SIZE>::print(this->DISTANCE_MATRIX);
  }

  void insertValue(double hue, double saturation);
  double distanceTo(const sure::HueDescriptor& rhs) const;

protected:

  static const std::vector<std::vector<double> > DISTANCE_MATRIX;

private:

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE> >(*this);
  }

};

} //

#endif /* HUE_DESCRIPTOR_H_ */
