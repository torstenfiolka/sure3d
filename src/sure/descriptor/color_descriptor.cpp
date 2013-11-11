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

#include "sure/descriptor/color_descriptor.h"

//! creates the earth mover's distance matrix for the color descriptor
std::vector<std::vector<double> > sure::descriptor::initColorDescriptorEMDMatrix()
{
  std::vector<std::vector<double> > matrix = sure::descriptor::initEMDMatrix<COLOR_DESCRIPTOR_SIZE+EXTRA_BIN_FOR_SATURATION_BALANCE>(sure::descriptor::MAX_EARTH_MOVERS_DISTANCE);
  for(unsigned int i=0; i<COLOR_DESCRIPTOR_SIZE; ++i)
  {
    matrix[COLOR_DESCRIPTOR_SIZE][i] = MAX_EARTH_MOVERS_DISTANCE;
    matrix[i][COLOR_DESCRIPTOR_SIZE] = MAX_EARTH_MOVERS_DISTANCE;
  }
  return matrix;
}

const std::vector<std::vector<double> > sure::descriptor::ColorDescriptor::DISTANCE_MATRIX = sure::descriptor::initColorDescriptorEMDMatrix();

void sure::descriptor::convertRGBtoHSL(Scalar r, Scalar g, Scalar b, Scalar& h, Scalar& s, Scalar& l)
{
  Scalar max = std::max(std::max(r,g), b);
  Scalar min = std::min(std::min(r,g), b);
  l = ((HistoType) (max+min)) * 0.5;

  if( fabs(max-min) < 1e-3 )
  {
    s = h = 0.0;
  }
  else
  {
    if( l < 0.5 )
    {
      s = (Scalar) ((max-min) / (max+min));
    }
    else
    {
      s = (Scalar) ((max-min) / (2.0-max-min));
    }

    if( r == max)
    {
      h = (Scalar) (1.0 + ((g-b) / (max-min)));
    }
    else if( g == max )
    {
      h = (Scalar) (3.0 + ((b-r) / (max-min)));
    }
    else if( b == max )
    {
      h = (Scalar) (5.0 + ((r-g) / (max-min)));
    }
  }
}


void sure::descriptor::ColorDescriptor::clear()
{
  sure::descriptor::ColorHistogram::clear();
  saturationBalance_ = 0.0;
}

void sure::descriptor::ColorDescriptor::normalize()
{
  saturationBalance_ /= weight_;
  sure::descriptor::ColorHistogram::normalize();
}

void sure::descriptor::ColorDescriptor::insertValue(Scalar hue, Scalar saturation)
{
  if( hue < MIN_HUE || hue > MAX_HUE || saturation < MIN_SATURATION || saturation > MAX_SATURATION )
  {
    return;
  }
  saturationBalance_ += MAX_SATURATION - saturation;
  weight_ += MAX_SATURATION - saturation;
  ColorHistogram::insertSmooth(hue, saturation, true);
}

sure::Scalar sure::descriptor::ColorDescriptor::distanceTo(const ColorDescriptor& rhs) const
{
  Scalar distance = 0.0;
  std::vector<double> lhsVec, rhsVec;
  ColorHistogram::fillVector(lhsVec);
  rhs.fillVector(rhsVec);
  lhsVec.push_back(saturationBalance_);
  rhsVec.push_back(rhs.saturationBalance_);
  distance += emd_hat<double>()(rhsVec, lhsVec, DISTANCE_MATRIX);
  return (distance * (1.0 / MAX_EARTH_MOVERS_DISTANCE));
}

std::ostream& sure::descriptor::operator<<(std::ostream& stream, const ColorDescriptor& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(2);
  stream << "Color " << (ColorHistogram) rhs;
  stream << std::setprecision(4);
  stream << "Saturation balance: " << rhs.saturationBalance_ << "\n";
  stream.unsetf(std::ios_base::fixed);
  return stream;
}

BOOST_CLASS_VERSION(sure::descriptor::ColorHistogram, 0)
BOOST_CLASS_VERSION(sure::descriptor::ColorDescriptor, 0)

