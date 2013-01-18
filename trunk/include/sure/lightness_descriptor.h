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

#ifndef LIGHTNESS_DESCRIPTOR_H_
#define LIGHTNESS_DESCRIPTOR_H_

#include "sure/histogram_emd.h"

namespace sure
{

class RelativeLightnessDescriptor : public sure::DescriptorHistogramWithEMDistance<10>
{
public:

  RelativeLightnessDescriptor() : sure::DescriptorHistogramWithEMDistance<10>(-1.0, 1.0)
  {
    referenceLightness = 0.0;
  }
  RelativeLightnessDescriptor(const sure::DescriptorHistogramWithEMDistance<10>& obj) : sure::DescriptorHistogramWithEMDistance<10>(obj)
  {
    referenceLightness = 0.0;
  }

  sure::RelativeLightnessDescriptor& operator=(const sure::DescriptorHistogramWithEMDistance<10>& rhs);
  sure::RelativeLightnessDescriptor operator +(const sure::RelativeLightnessDescriptor& rhs) const;
  sure::RelativeLightnessDescriptor& operator +=(const sure::RelativeLightnessDescriptor& rhs);
  sure::RelativeLightnessDescriptor operator *(const double rhs) const;
  sure::RelativeLightnessDescriptor& operator *=(const double rhs);
  sure::RelativeLightnessDescriptor operator /(const double rhs) const;
  sure::RelativeLightnessDescriptor& operator /=(const double rhs);

  virtual void print() const;

  void setLightness(double lightness) { this->referenceLightness = lightness; }
  double getLightness() const { return referenceLightness; }

  virtual void insertValue(double lightness);

protected:

  double referenceLightness;

};

}

#endif /* LIGHTNESS_DESCRIPTOR_H_ */
