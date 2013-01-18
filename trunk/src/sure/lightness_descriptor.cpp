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

#include "sure/lightness_descriptor.h"

sure::RelativeLightnessDescriptor& sure::RelativeLightnessDescriptor::operator=(const sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>& rhs)
{
  if( this != &rhs )
  {
    sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>::operator =(rhs);
  }
  return *this;
}

sure::RelativeLightnessDescriptor sure::RelativeLightnessDescriptor::operator +(const sure::RelativeLightnessDescriptor& rhs) const
{
  sure::RelativeLightnessDescriptor lhs(*this);
  lhs = ((sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>) lhs).operator +=(rhs);
  lhs.referenceLightness = (lhs.referenceLightness + rhs.referenceLightness) / 2.0;
  return lhs;
}

sure::RelativeLightnessDescriptor& sure::RelativeLightnessDescriptor::operator +=(const sure::RelativeLightnessDescriptor& rhs)
{
  *this = ((sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>) *this).operator +=(rhs);
  this->referenceLightness = (this->referenceLightness + rhs.referenceLightness) / 2.0;
  return *this;
}

sure::RelativeLightnessDescriptor sure::RelativeLightnessDescriptor::operator *(const double rhs) const
{
  sure::RelativeLightnessDescriptor lhs(*this);
  lhs = ((sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>) lhs).operator *=(rhs);
  return lhs;
}

sure::RelativeLightnessDescriptor& sure::RelativeLightnessDescriptor::operator *=(const double rhs)
{
  *this = ((sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>) *this).operator *=(rhs);
  return *this;
}

sure::RelativeLightnessDescriptor sure::RelativeLightnessDescriptor::operator /(const double rhs) const
{
  sure::RelativeLightnessDescriptor lhs(*this);
  lhs = ((sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>) lhs).operator /=(rhs);
  return lhs;
}

sure::RelativeLightnessDescriptor& sure::RelativeLightnessDescriptor::operator /=(const double rhs)
{
  *this = ((sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>) *this).operator /=(rhs);
  return *this;
}

void sure::RelativeLightnessDescriptor::print() const
{
  sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>::print();
  std::cout << std::fixed << std::setprecision(3) << "[RelativeLightnessDescriptor] " << name << ": Reference Lightness: " << referenceLightness << std::endl;
}

void sure::RelativeLightnessDescriptor::insertValue(double lightness)
{
  sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>::insertValue(lightness - referenceLightness);
}

BOOST_CLASS_VERSION(sure::DescriptorHistogramWithEMDistance<sure::LIGHTNESS_DESCRIPTOR_SIZE>, 0)
BOOST_CLASS_VERSION(sure::RelativeLightnessDescriptor, 0)
