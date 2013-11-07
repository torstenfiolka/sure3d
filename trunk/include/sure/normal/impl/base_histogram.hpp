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


template <int HistogramSize>
std::ostream& sure::normal::operator<<(std::ostream& stream, const sure::normal::BaseHistogram<HistogramSize>& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(2);

  std::cout << "NormalHistogram: ";
  for(int i=0; i<HistogramSize; ++i)
  {
    std::cout << rhs.values_[i] << " ";
  }
  std::cout << "\nPoints: " << rhs.numberOfEntries_ << " Weight: " << rhs.weight_;
  std::cout << "\n";
  stream.unsetf(std::ios_base::fixed);
  return stream;
}


template <int HistogramSize>
void sure::normal::BaseHistogram<HistogramSize>::clear()
{
  numberOfEntries_ = 0;
  weight_ = 0.f;
  for(int i=0; i<HistogramSize; ++i)
  {
    values_[i] = 0.f;
  }
}

//template <int HistogramSize>
//void sure::normal::BaseHistogram<HistogramSize>::calculateHistogram(const sure::NormalType& normal)
//{
//  this->clear();
//  numberOfEntries_ = 1;
//  float distance = 0.f, totalSum = 0.f, weightedValue;
//  for(int i=0; i<this->HISTOGRAM_SIZE; ++i)
//  {
//    distance = (normal.dot(sure::normal::REFERENCE_VECTORS[i]));
//    if( distance > this->MAX_INFLUENCE_DISTANCE )
//    {
//      weightedValue = (((distance - this->MAX_INFLUENCE_DISTANCE) / (1.f - this->MAX_INFLUENCE_DISTANCE)));
//      totalSum += weightedValue;
//      values_[i] = weightedValue;
//    }
//    else
//    {
//      values_[i] = 0.f;
//    }
//  }
//  if( totalSum > 0.f )
//  {
//    for(int i=0; i<this->HISTOGRAM_SIZE; ++i)
//    {
//      values_[i] = (values_[i] / totalSum);
//      weight_ += values_[i];
//    }
//  }
//}
//
//template <int HistogramSize>
//void sure::normal::BaseHistogram<HistogramSize>::insertCrossProduct(const NormalType& referenceNormal, const NormalType& secondNormal)
//{
//  NormalType cpVector = (referenceNormal.cross(secondNormal)).normalized();
//
//  Scalar dotProduct = referenceNormal.dot(secondNormal);
//  HistoType crossProductWeight = 1.f - fabs(dotProduct);
//
//  for(int i=0; i<this->HISTOGRAM_SIZE; ++i)
//  {
//    HistoType distance = (cpVector.dot(sure::normal::REFERENCE_VECTORS[i]));
//    if( distance > this->MAX_INFLUENCE_DISTANCE )
//    {
//      HistoType weightedValue = (((distance - this->MAX_INFLUENCE_DISTANCE) / (1.f - this->MAX_INFLUENCE_DISTANCE))) * crossProductWeight;
//      values_[i] += weightedValue;
//      this->weight_ += weightedValue;
//    }
//  }
//
//  this->weight_ += 1.f - crossProductWeight;
//  zeroClass_ += 1.f - crossProductWeight;
//  numberOfEntries_++;
//}

template <int HistogramSize>
sure::HistoType sure::normal::BaseHistogram<HistogramSize>::calculateRawEntropy() const
{
  HistoType entropy(0.f);
  if( weight_ > 0.f )
  {
    HistoType sum = 0.f;
    for(int i=0; i<HistogramSize; ++i)
    {
      HistoType tempValue = values_[i] / weight_;
      if( tempValue > EPSILON )
      {
        sum += (tempValue) * (log(tempValue) / LOG_BASE_2);
      }
    }
    entropy = -sum;
  }
  return entropy;
}

template <int HistogramSize>
pcl::Histogram<HistogramSize> sure::normal::BaseHistogram<HistogramSize>::getHistogram() const
{
  pcl::Histogram<HistogramSize> h;
  for(int i=0; HistogramSize; ++i)
  {
      h.histogram[i] = values_[i];
  }
  return h;
}
