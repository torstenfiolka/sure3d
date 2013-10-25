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

#include "sure/histogram_l2.h"

const std::string sure::DescriptorHistogramWithL2Distance::HISTOGRAM_NAME = std::string("L2_Histogram");

sure::DescriptorHistogramWithL2Distance::DescriptorHistogramWithL2Distance(double min, double max, unsigned int histoSize, const std::string& name) : sure::HistogramBase(histoSize, min, max, name), histogram(NULL)
{
  if( histogramSize > 0 )
  {
    this->histogram = new double[histogramSize];
    clear();
  }
}

sure::DescriptorHistogramWithL2Distance::DescriptorHistogramWithL2Distance(const sure::HistogramBase& obj) : sure::HistogramBase(obj)
{
  if( histogramSize > 0 )
  {
    this->histogram = new double[histogramSize];
    clear();
  }
}

sure::DescriptorHistogramWithL2Distance::~DescriptorHistogramWithL2Distance()
{
  if( histogram )
  {
    delete[] histogram;
    histogram = NULL;
  }
}

sure::DescriptorHistogramWithL2Distance& sure::DescriptorHistogramWithL2Distance::operator=(const sure::DescriptorHistogramWithL2Distance& rhs)
{
  if( this != &rhs )
  {
    sure::HistogramBase::operator =(rhs);
    if( rhs.histogramSize > 0 && rhs.histogram )
    {
      this->histogram = new double[histogramSize];
      if( histogram )
      {
        for(unsigned int i=0; i<histogramSize; ++i)
        {
          histogram[i] = rhs.histogram[i];
        }
      }
    }
  }
  return *this;
}

sure::DescriptorHistogramWithL2Distance& sure::DescriptorHistogramWithL2Distance::operator=(const sure::HistogramBase& rhs)
{
  if( this != &rhs )
  {
    sure::HistogramBase::operator =(rhs);
  }
  return *this;
}

sure::DescriptorHistogramWithL2Distance sure::DescriptorHistogramWithL2Distance::operator +(const sure::DescriptorHistogramWithL2Distance& rhs) const
{
  sure::DescriptorHistogramWithL2Distance lhs(*this);
  lhs = ((sure::HistogramBase) lhs).operator +=(rhs);
  if( lhs.histogram && rhs.histogram )
  {
    for(unsigned int i=0; i<lhs.histogramSize; ++i)
    {
      lhs.histogram[i] += rhs.histogram[i];
    }
  }
  return lhs;
}

sure::DescriptorHistogramWithL2Distance& sure::DescriptorHistogramWithL2Distance::operator +=(const sure::DescriptorHistogramWithL2Distance& rhs)
{
  *this = ((sure::HistogramBase) *this).operator +=(rhs);
  if( this->histogram && rhs.histogram )
  {
    for(unsigned int i=0; i<histogramSize; ++i)
    {
      this->histogram[i] += rhs.histogram[i];
    }
  }
  return *this;
}

sure::DescriptorHistogramWithL2Distance sure::DescriptorHistogramWithL2Distance::operator *(const double rhs) const
{
  sure::DescriptorHistogramWithL2Distance lhs(*this);
  lhs = ((sure::HistogramBase) lhs).operator *=(rhs);
  if( lhs.histogram )
  {
    for(unsigned int i=0; i<histogramSize; ++i)
    {
      lhs.histogram[i] *= rhs;
    }
  }
  return lhs;
}

sure::DescriptorHistogramWithL2Distance& sure::DescriptorHistogramWithL2Distance::operator *=(const double rhs)
{
  *this = ((sure::HistogramBase) *this).operator *=(rhs);
  if( this->histogram )
  {
    for(unsigned int i=0; i<histogramSize; ++i)
    {
      this->histogram[i] *= rhs;
    }
  }
  return *this;
}

sure::DescriptorHistogramWithL2Distance sure::DescriptorHistogramWithL2Distance::operator /(const double rhs) const
{
  sure::DescriptorHistogramWithL2Distance lhs(*this);
  lhs = ((sure::HistogramBase) lhs).operator /=(rhs);
  if( lhs.histogram )
  {
    for(unsigned int i=0; i<histogramSize; ++i)
    {
      lhs.histogram[i] /= rhs;
    }
  }
  return lhs;
}

sure::DescriptorHistogramWithL2Distance& sure::DescriptorHistogramWithL2Distance::operator /=(const double rhs)
{
  *this = ((sure::HistogramBase) *this).operator /=(rhs);
  if( this->histogram )
  {
    for(unsigned int i=0; i<histogramSize; ++i)
    {
      this->histogram[i] /= rhs;
    }
  }
  return *this;
}

void sure::DescriptorHistogramWithL2Distance::DescriptorHistogramWithL2Distance::clear()
{
  sure::HistogramBase::clear();
  if( histogram )
  {
    for(unsigned int i=0; i<histogramSize; ++i)
    {
      histogram[i] = 0.0;
    }
  }
}

void sure::DescriptorHistogramWithL2Distance::normalize()
{
  if( histogramWeight > 0.0 )
  {
    for(unsigned int j=0; j<histogramSize; ++j)
    {
      histogram[j] = histogram[j] / (double) histogramWeight;
    }
    histogramWeight = 1.0;
  }
}

void sure::DescriptorHistogramWithL2Distance::print() const
{
  sure::HistogramBase::print();
  double sum = 0.0;
  std::cout << "[L2_Histogram] " << name << ": ";
  for(unsigned int j=0; j<histogramSize; ++j)
  {
    std::cout << std::fixed << std::setprecision(3) << histogram[j] << " ";
    sum += histogram[j];
  }
  std::cout << std::fixed << std::setprecision(3) << "Sum: " << sum << std::endl;
}

void sure::DescriptorHistogramWithL2Distance::insertValue(double value)
{
  if( value < minimum || value > maximum )
  {
    return;
  }
  int targetBin = floor(((value - minimum) / binSize));
  double distance, weight = 0.0;

  for(int i=targetBin-1; i<=targetBin+1; ++i)
  {
    if( i < 0 || i >= (int) histogramSize )
    {
      continue;
    }
    distance = fabs((value - minimum) - ((0.5 + (double) i) * binSize ));
    if( distance < binSize )
    {
      histogram[i] += 1.0 - (distance / binSize);
      weight += 1.0 - (distance / binSize);
    }
  }
  histogramWeight += weight;
  numberOfPoints++;
}

double sure::DescriptorHistogramWithL2Distance::distanceTo(const sure::DescriptorHistogramWithL2Distance& rhs) const
{
  if( !this->isNormalized() || !rhs.isNormalized() || this->histogramSize != rhs.histogramSize )
  {
    return INFINITY;
  }
  double distance = 0.0;
  for(unsigned int j=0; j<histogramSize; ++j)
  {
    distance += (this->histogram[j] - rhs.histogram[j]) * (this->histogram[j] - rhs.histogram[j]);
  }
  if( distance > 0.0 )
  {
    distance = sqrt(distance);
  }
  return (distance * (1.0 / (double) this->MAX_DISTANCE));
}

void sure::DescriptorHistogramWithL2Distance::fillRandom()
{
  sure::HistogramBase::clear();
  for(unsigned int j=0; j<histogramSize; ++j)
  {
    histogram[j] = (double) rand() / (double) RAND_MAX;
    histogramWeight += histogram[j];
  }
  this->normalize();
}

BOOST_CLASS_VERSION(sure::DescriptorHistogramWithL2Distance, 0)
