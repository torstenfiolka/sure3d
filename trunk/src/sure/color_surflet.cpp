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

#include "sure/color_surflet.h"

void sure::convertPCLRGBtoFloatRGB(float rgb, float& r, float& g, float& b)
{
  int color = *reinterpret_cast<const int*>(&rgb);
  r = float((0xff0000 & color) >> 16) / 255.f;
  g = float((0x00ff00 & color) >> 8) / 255.f;
  b = float( 0x0000ff & color) / 255.f;
}

void sure::convertRGBtoHSL(const float r, const float g, const float b, double& h, double& s, double& l)
{
  float max = std::max(std::max(r,g), b);
  float min = std::min(std::min(r,g), b);
  l = ((double) (max+min)) * 0.5;

  if( fabs(max-min) < 0.0001f )
  {
    s = h = 0.0;
  }
  else
  {
    if( l < 0.5 )
    {
      s = (double) ((max-min) / (max+min));
    }
    else
    {
      s = (double) ((max-min) / (2.f-max-min));
    }

    if( r == max)
    {
      h = (double) (1.f + ((g-b) / (max-min)));
    }
    else if( g == max )
    {
      h = (double) (3.f + ((b-r) / (max-min)));
    }
    else if( b == max )
    {
      h = (double) (5.f + ((r-g) / (max-min)));
    }
  }
}

std::ostream& sure::operator<<(std::ostream& stream, const sure::ColorSurflet& rhs)
{
  stream << (sure::Surflet) rhs;
  stream << std::fixed << std::setprecision(3) << "[ColorSurflet] RGB: " << rhs.r << " / " << rhs.g << " / " << rhs.b << std::endl;
  return stream;
}
