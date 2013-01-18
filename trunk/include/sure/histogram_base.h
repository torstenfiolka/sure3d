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

#ifndef HISTOGRAM_BASE_H_
#define HISTOGRAM_BASE_H_

#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>

namespace sure
{

class HistogramBase
{
public:

  HistogramBase(unsigned int size, double min, double max, const std::string& name)
  {
    histogramSize = size;
    minimum = min;
    maximum = max;
    this->name = name;
    range = maximum - minimum;
    binSize = range / (double) (histogramSize);
    clear();
  }

  HistogramBase(const sure::HistogramBase& obj)
  {
    this->operator=(obj);
  }

  virtual ~HistogramBase() {}

  virtual void clear()
  {
    histogramWeight = 0.0;
    numberOfPoints = 0;
  }

  sure::HistogramBase& operator=(const sure::HistogramBase& rhs)
  {
    if( this != &rhs )
    {
      this->histogramSize = rhs.histogramSize;
      this->numberOfPoints = rhs.numberOfPoints;
      this->histogramWeight = rhs.histogramWeight;
      this->name = rhs.name;
      this->minimum = rhs.minimum;
      this->maximum = rhs.maximum;
      this->binSize = rhs.binSize;
      this->range = rhs.range;
    }
    return *this;
  }

  sure::HistogramBase operator +(const sure::HistogramBase& rhs) const
  {
    sure::HistogramBase lhs(*this);
    lhs.numberOfPoints += rhs.numberOfPoints;
    lhs.histogramWeight += rhs.histogramWeight;
    return lhs;
  }

  sure::HistogramBase& operator +=(const sure::HistogramBase& rhs)
  {
    this->numberOfPoints += rhs.numberOfPoints;
    this->histogramWeight += rhs.histogramWeight;
    return *this;
  }

  sure::HistogramBase operator *(const double rhs) const
  {
    sure::HistogramBase lhs(*this);
    lhs.histogramWeight *= rhs;
    return lhs;
  }

  sure::HistogramBase& operator *=(const double rhs)
  {
    this->histogramWeight *= rhs;
    return *this;
  }

  sure::HistogramBase operator /(const double rhs) const
  {
    sure::HistogramBase lhs(*this);
    lhs.histogramWeight /= rhs;
    return lhs;
  }

  sure::HistogramBase& operator /=(const double rhs)
  {
    this->histogramWeight /= rhs;
    return *this;
  }

  bool isNormalized() const
  {
    return (histogramWeight == 1.0);
  }

  const std::string& getName() const { return name; }
  unsigned int getSize() const { return histogramSize; }

  virtual void print() const
  {
    std::cout << std::fixed << std::setprecision(3) << "[HistogramBase] " << name << " Histogram Size: " << histogramSize << " Bin Size: " << binSize << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "[HistogramBase] " << name << " Min: " << minimum << " Max: " << maximum << " Range: " << range << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "[HistogramBase] " << name << " Histogram Weight: " << histogramWeight << " Number of Points: " << numberOfPoints << " Normalized: " << isNormalized() << std::endl;
  }

  double distanceTo(const sure::HistogramBase& rhs) { return INFINITY; }

protected:

  std::string name;

  double minimum, maximum, range, binSize, histogramWeight;
  unsigned int histogramSize, numberOfPoints;

};


} // namespace

#endif /* HISTOGRAM_BASE_H_ */
