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

#include <sure/normal/normal_histogram.h>

const sure::HistoType sure::normal::NormalHistogram::MAX_ENTROPY = log(NORMAL_HISTOGRAM_SIZE) / log(2.f);

void sure::normal::NormalHistogram::insertNormal(const sure::NormalType& normal)
{
  this->clear();
  numberOfEntries_ = 1;
  float distance = 0.f, totalSum = 0.f, weightedValue;
  for(int i=0; i<NORMAL_HISTOGRAM_SIZE; ++i)
  {
    distance = (normal.dot(sure::normal::REFERENCE_VECTORS[i]));
    if( distance > influenceRadius_ )
    {
      weightedValue = (((distance - influenceRadius_) / (1.f - influenceRadius_)));
      totalSum += weightedValue;
      values_[i] = weightedValue;
    }
    else
    {
      values_[i] = 0.f;
    }
  }
  if( totalSum > 0.f )
  {
    for(int i=0; i<NORMAL_HISTOGRAM_SIZE; ++i)
    {
      values_[i] = (values_[i] / totalSum);
      weight_ += values_[i];
    }
  }
}
