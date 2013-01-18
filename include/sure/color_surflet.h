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

#ifndef COLOR_SURFLET_H_
#define COLOR_SURFLET_H_

#include "sure/surflet.h"

namespace sure
{

//! converts a pcl-float value to thre floats
void convertPCLRGBtoFloatRGB(float rgb, float& r, float& g, float& b);

//! converts a RGB Color to the HSL colorspace
void convertRGBtoHSL(const float r, const float g, const float b, double& h, double& s, double& l);

class ColorSurflet : public sure::Surflet
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  float r, g, b;

  ColorSurflet()
  {
    this->initialize();
  }

  ColorSurflet(const Eigen::Vector3f pos) : sure::Surflet(pos)
  {
    this->initialize();
  }

  ColorSurflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal) : sure::Surflet(pos, normal)
  {
    this->initialize();
  }

  ColorSurflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal, float pclRGB)
  {
    sure::convertPCLRGBtoFloatRGB(pclRGB, r, g, b);
    this->point = pos;
    this->normal = normal;
  }

  ColorSurflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal, float r, float g, float b)
  {
    this->r = r;
    this->g = g;
    this->b = b;
    this->point = pos;
    this->normal = normal;
  }

  ColorSurflet(const sure::Surflet& surflet) : sure::Surflet(surflet)
  {
    this->initialize();
  }

  ColorSurflet(const sure::Surflet& surflet, float r, float g, float b) : sure::Surflet(surflet)
  {
    this->r = r;
    this->g = g;
    this->b = b;
  }


  virtual void reset()
  {
    sure::Surflet::reset();
    this->initialize();
  }

  virtual void initialize()
  {
    r = g = b = INFINITY;
  }

  virtual void print() const
  {
    sure::Surflet::print();
    std::cout << std::fixed << std::setprecision(3) << "[ColorSurflet] RGB: " << r << " / " << g << " / " << b << std::endl;
  }

  virtual ~ColorSurflet() { }

  void setColor(float pclRGB)
  {
    sure::convertPCLRGBtoFloatRGB(pclRGB, r, g, b);
  }

  void setColor(float r, float g, float b)
  {
    this->r = r;
    this->g = g;
    this->b = b;
  }

//  friend std::ostream& operator<<(std::ostream& stream, const sure::ColorSurflet& rhs);

private:

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<sure::Surflet>(*this);
    ar & r;
    ar & g;
    ar & b;
  }

};

//std::ostream& operator<<(std::ostream& stream, const sure::ColorSurflet& rhs);

}

#endif /* COLOR_SURFLET_H_ */
