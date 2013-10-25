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

#include <sure/normal_histogram.h>

const float sure::NormalHistogram::LOG_BASE_2 = log(2.f);
const float sure::NormalHistogram::MAX_ENTROPY = log(sure::NormalHistogram::HISTOGRAM_SIZE) / log(2.f);
const float sure::NormalHistogram::MAX_DISTANCE = cos(M_PI / 3.f);

sure::NormalHistogram& sure::NormalHistogram::operator=(const sure::NormalHistogram& rhs)
{
  if( this != &rhs )
  {
    this->entropy = rhs.entropy;
    this->numberOfNormals = rhs.numberOfNormals;
    this->weight = rhs.weight;
    this->zeroClass = rhs.zeroClass;
    for(int i=0; i<HISTOGRAM_SIZE; ++i)
    {
      this->values[i] = rhs.values[i];
    }
  }
  return *this;
}

sure::NormalHistogram sure::NormalHistogram::operator+(const sure::NormalHistogram& rhs) const
{
  sure::NormalHistogram h = *this;
  h.numberOfNormals += rhs.numberOfNormals;
  h.weight += rhs.weight;
  for(int i=0; i<HISTOGRAM_SIZE; ++i)
  {
    h.values[i] += rhs.values[i];
  }
  return h;
}

sure::NormalHistogram& sure::NormalHistogram::operator+=(const sure::NormalHistogram& rhs)
{
  this->weight += rhs.weight;
  this->numberOfNormals += rhs.numberOfNormals;
  for(int i=0; i<HISTOGRAM_SIZE; ++i)
  {
    this->values[i] += rhs.values[i];
  }
  return *this;
}

sure::NormalHistogram sure::NormalHistogram::operator*(const float& rhs) const
{
  sure::NormalHistogram h = *this;
  h.weight = rhs;
  for(int i=0; i<HISTOGRAM_SIZE; ++i)
  {
    h.values[i] *= rhs;
  }
  return h;
}

void sure::NormalHistogram::clear()
{
  numberOfNormals = 0;
  entropy = 0.f;
  weight = 0.f;
  for(int i=0; i<HISTOGRAM_SIZE; ++i)
  {
    values[i] = 0.f;
  }
  zeroClass = 0.f;
}

void sure::NormalHistogram::calculateHistogram(const Eigen::Vector3f& normal)
{
  this->clear();
  numberOfNormals = 1;
  float distance = 0.f, totalSum = 0.f, weightedValue;
  for(int i=0; i<this->HISTOGRAM_SIZE; ++i)
  {
    distance = (normal.dot(sure::REFERENCE_VECTORS.at(i)));
//    if( distance >= 1.f )
//    {
//      distance = 0.9999f;
//    }
//    if( distance <= -1.f )
//    {
//      distance = -0.9999f;
//    }
//    distance = acos(distance);
    if( distance > this->MAX_DISTANCE )
    {
      weightedValue = (((distance - this->MAX_DISTANCE) / (1.f - this->MAX_DISTANCE)));
      totalSum += weightedValue;
      values[i] = weightedValue;
    }
    else
    {
      values[i] = 0.f;
    }
  }
  if( totalSum > 0.f )
  {
    for(int i=0; i<this->HISTOGRAM_SIZE; ++i)
    {
      values[i] = (values[i] / totalSum);
      weight += values[i];
    }
  }
}

void sure::NormalHistogram::insertCrossProduct(const Eigen::Vector3f& referenceNormal, const Eigen::Vector3f& secondNormal, sure::CrossProductWeightMethod weightMethod)
{
  Eigen::Vector3f cpVector = (referenceNormal.cross(secondNormal)).normalized();

  float dotProduct = referenceNormal.dot(secondNormal);
  float crossProductWeight = 1.f;
  float distance, weightedValue;

  switch( weightMethod )
  {
    case sure::EXCLUSION:
      if( dotProduct > 0.9f )
        return;
      break;
    case sure::INVERSE_ABSOLUTE_DOT_PRODUCT:
      crossProductWeight = 1.f - fabs(dotProduct);
      break;
    case sure::INVERSE_POSITIVE_DOT_PRODUCT:
      crossProductWeight = std::min(1.f, 1.f - dotProduct);
      break;
    default:
    case sure::NO_WEIGHT:
      break;
  }

  for(int i=0; i<this->HISTOGRAM_SIZE; ++i)
  {
    distance = (cpVector.dot(sure::REFERENCE_VECTORS.at(i)));
//    if( distance >= 1.f )
//    {
//      distance = 1.f - this->EPSILON;
//    }
//    if( distance <= -1.f )
//    {
//      distance = -1.f + this->EPSILON;
//    }
//    distance = acosf(distance);
    if( distance > this->MAX_DISTANCE )
    {
      weightedValue = (((distance - this->MAX_DISTANCE) / (1.f - this->MAX_DISTANCE))) * crossProductWeight;
      values[i] += weightedValue;
      this->weight += weightedValue;
    }
  }

  this->weight += 1.f - crossProductWeight;
  zeroClass += 1.f - crossProductWeight;
  numberOfNormals++;
}


void sure::NormalHistogram::calculateHistogram(const float normal[3])
{
  Eigen::Vector3f vec(normal[0], normal[1], normal[2]);
  calculateHistogram(vec);
}

void sure::NormalHistogram::calculateEntropy()
{
  if( weight > 0.f )
  {
    float sum = 0.f;
    float tempValue = 0.f;
    for(int i=0; i<HISTOGRAM_SIZE; ++i)
    {
      tempValue = values[i] / weight;
      if( tempValue > this->EPSILON )
      {
        sum += (tempValue) * (log(tempValue) / LOG_BASE_2);
      }
    }
    if( zeroClass > 0.f )
    {
      tempValue = zeroClass / weight;
      if( tempValue > this->EPSILON )
      {
        sum += (tempValue) * (log(tempValue) / LOG_BASE_2);
      }
    }
    entropy= -sum / MAX_ENTROPY;
    if( !std::isfinite(entropy) )
    {
      entropy = 0.f;
    }
  }
  else
  {
    entropy = 0.f;
  }
}

pcl::Histogram<sure::NormalHistogram::HISTOGRAM_SIZE> sure::NormalHistogram::getHistogram() const
{
  pcl::Histogram<HISTOGRAM_SIZE> h;
  for(int i=0; HISTOGRAM_SIZE; ++i)
  {
      h.histogram[i] = values[i];
  }
  return h;
}

void sure::NormalHistogram::print() const
{
  std::cout << "Histogram: ";
  for(int i=0; i<sure::NormalHistogram::HISTOGRAM_SIZE; ++i)
  {
    std::cout << values[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "Points: " << numberOfNormals << " Weight: " << weight << " Entropy: " << entropy << std::endl;
}

