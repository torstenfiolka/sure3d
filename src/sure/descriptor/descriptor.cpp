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

#include <sure/descriptor/descriptor.h>

void sure::descriptor::Descriptor::clear()
{
  shape_.clear();
  color_.clear();
  lightness_.clear();
}

bool sure::descriptor::Descriptor::isNormalized() const
{
  return (shape_.isNormalized() && color_.isNormalized() && lightness_.isNormalized());
}

void sure::descriptor::Descriptor::normalize()
{
  shape_.normalize();
  color_.normalize();
  lightness_.normalize();
}

sure::Scalar sure::descriptor::Descriptor::distanceTo(const Descriptor& rhs, Scalar shapeWeight, Scalar colorWeight, Scalar lightnessWeight) const
{
  Scalar distance = 0.0;
  distance += shape_.distanceTo(rhs.shape_) * shapeWeight;
  distance += color_.distanceTo(rhs.color_) * colorWeight;
  distance += lightness_.distanceTo(rhs.lightness_) * lightnessWeight;

  return distance / (shapeWeight + colorWeight + lightnessWeight);
}

//template <int SplitN>
//void sure::descriptor::Descriptor::createRandomDescriptor()
//{
//  for(unsigned int i=0; i<numberOfDescriptors; ++i)
//  {
//    pfDescriptor[i].fillRandom();
//    colorDescriptor[i].fillRandom();
//    lightnessDescriptor[i].fillRandom();
//  }
//}

std::ostream& sure::descriptor::operator<<(std::ostream& stream, const Descriptor& rhs)
{
  stream << rhs.shape_;
  stream << rhs.color_;
  stream << rhs.lightness_;
  return stream;
}

BOOST_CLASS_VERSION(sure::descriptor::Descriptor, 0)
