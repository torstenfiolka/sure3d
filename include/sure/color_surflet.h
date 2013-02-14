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


  ColorSurflet() : mRed(0.f), mGreen(0.f), mBlue(0.f) { }
  ColorSurflet(const Eigen::Vector3f pos) : sure::Surflet(pos), mRed(0.f), mGreen(0.f), mBlue(0.f) { }
  ColorSurflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal) : sure::Surflet(pos, normal), mRed(0.f), mGreen(0.f), mBlue(0.f) { }
  ColorSurflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal, float pclRGB) : sure::Surflet(pos, normal), mRed(0.f), mGreen(0.f), mBlue(0.f)
  {
    sure::convertPCLRGBtoFloatRGB(pclRGB, mRed, mGreen, mBlue);
  }
  ColorSurflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal, float r, float g, float b) : sure::Surflet(pos, normal), mRed(r), mGreen(g), mBlue(b) { }
  ColorSurflet(const sure::Surflet& surflet) : sure::Surflet(surflet), mRed(0.f), mGreen(0.f), mBlue(0.f) { }
  ColorSurflet(const sure::Surflet& surflet, float r, float g, float b) : sure::Surflet(surflet), mRed(r), mGreen(g), mBlue(b) { }

  virtual void reset()
  {
    sure::Surflet::reset();
    mRed = mGreen = mBlue = 0.f;
  }

  virtual void print() const
  {
    sure::Surflet::print();
    std::cout << std::fixed << std::setprecision(3) << "[ColorSurflet] RGB: " << mRed << " / " << mGreen << " / " << mBlue << std::endl;
  }

  virtual ~ColorSurflet() { }

  void setColor(float pclRGB)
  {
    sure::convertPCLRGBtoFloatRGB(pclRGB, mRed, mGreen, mBlue);
  }
  void setColor(float r, float g, float b)
  {
    this->mRed = r;
    this->mGreen = g;
    this->mBlue = b;
  }

  float r() const { return mRed; }
  float g() const { return mGreen; }
  float b() const { return mBlue; }

  //  friend std::ostream& operator<<(std::ostream& stream, const sure::ColorSurflet& rhs);

protected:

  float mRed, mGreen, mBlue;

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
