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

#include "sure/hue_descriptor.h"

const std::string sure::HueDescriptor::HISTOGRAM_NAME = std::string("Hue Histogram");

const double sure::HueDescriptor::MAX_SATURATION_DISTANCE = sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::MAX_DISTANCE;

//! creates the earth mover's distance matrix for the color descriptor
std::vector<std::vector<double> > sure::initHueEMDMatrix()
{
  std::vector<std::vector<double> > matrix;
  matrix.resize(sure::COLOR_HISTOGRAM_SIZE);
  if( sure::COLOR_HISTOGRAM_SIZE > 0 )
  {
    for(unsigned int i=0; i<sure::COLOR_HISTOGRAM_SIZE-1; ++i)
    {
      matrix[i].resize(sure::COLOR_HISTOGRAM_SIZE, 0.0);
      for(unsigned int j=0; j<sure::COLOR_HISTOGRAM_SIZE-1; ++j)
      {
        matrix[i][j] = std::min(std::min((double) abs(i-j),(double) (sure::COLOR_HISTOGRAM_SIZE-1)-abs(i-j)), sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::MAX_DISTANCE);
      }
    }
    matrix[sure::COLOR_HISTOGRAM_SIZE-1].resize(sure::COLOR_HISTOGRAM_SIZE, 0.0);
    for(unsigned int i=0; i<sure::COLOR_HISTOGRAM_SIZE-1; ++i)
    {
      matrix[sure::COLOR_HISTOGRAM_SIZE-1][i] = sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::MAX_DISTANCE;
      matrix[i][sure::COLOR_HISTOGRAM_SIZE-1] = sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::MAX_DISTANCE;
    }
  }
  return matrix;
}

sure::HueDescriptor& sure::HueDescriptor::operator=(const sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>& obj)
{
  if( this != &obj )
  {
    sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::operator =(obj);
  }
  return *this;
}

void sure::HueDescriptor::insertValue(double hue, double saturation)
{
  if( hue < sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::minimum || hue > sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::maximum )
  {
//    ROS_WARN("Value discarded: %f/%f", hue, saturation);
    return;
  }

  int index, targetBin = floor((hue - sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::minimum) / sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::binSize);
  double tempHue, distance, partialSaturation, insertedSum = 0.0;

  for(int i=targetBin-1; i<=targetBin+1; ++i)
  {
    index = i;
    tempHue = hue;
    if( i < 0 )
    {
      index += (sure::COLOR_HISTOGRAM_SIZE - 1);
      tempHue += MAX_HUE;
    }
    else if( i >= (int) (sure::COLOR_HISTOGRAM_SIZE - 1) )
    {
      index -= (sure::COLOR_HISTOGRAM_SIZE - 1);
      tempHue -= MAX_HUE;
    }
    distance = fabs(tempHue - (((double)index+0.5) * sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::binSize ));
    if( distance < sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::binSize )
    {
      insertedSum += partialSaturation = saturation * (1.0 - (distance / sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::binSize));
      sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::histogram[index] += partialSaturation;
    }
  }
  if( insertedSum > 1.0 )
  {
//    ROS_WARN("Summe > 1");
  }
  sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::histogram[sure::COLOR_HISTOGRAM_SIZE-1] += 1.0 - insertedSum;
  sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::histogramWeight += 1.0;
  sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::numberOfPoints++;
}

double sure::HueDescriptor::distanceTo(const sure::HueDescriptor& rhs) const
{
  if( !this->isNormalized() || !rhs.isNormalized() )
  {
    return INFINITY;
  }
  double distance = 0.0;
  distance += emd_hat<double>()(this->histogram, rhs.histogram, this->DISTANCE_MATRIX);
  return (distance * (1.0 / (double) sure::DescriptorHistogramWithEMDistance<sure::COLOR_HISTOGRAM_SIZE>::MAX_DISTANCE));
}

const std::vector<std::vector<double> > sure::HueDescriptor::DISTANCE_MATRIX = sure::initHueEMDMatrix();


