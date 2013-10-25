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

#ifndef SURFLET_H_
#define SURFLET_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include <iostream>
#include <iomanip>

#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/base_object.hpp>

namespace sure
{

class Surflet
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Surflet() : mNormal(Eigen::Vector3f::Zero()), mPosition(Eigen::Vector3f::Zero()) {  }

  Surflet(const Eigen::Vector3f pos) : mNormal(Eigen::Vector3f::Zero()), mPosition(pos) {  }

  Surflet(const Eigen::Vector3f pos, const Eigen::Vector3f mNormal) : mNormal(mNormal), mPosition(pos) { }

  virtual void reset()
  {
    mNormal = Eigen::Vector3f::Zero();
    mPosition = Eigen::Vector3f::Zero();
  }

  virtual void print() const
  {
    std::cout << std::fixed << std::setprecision(3) << "[Surflet] Position: " << mPosition[0] << " / " << mPosition[1] << " / " << mPosition[2] << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "[Surflet] Normal: " << mNormal[0] << " / " << mNormal[1] << " / " << mNormal[2] << std::endl;
  }

  void setNormal(const Eigen::Vector3f& normal) { mNormal = normal; }
  void setNormal(float x, float y, float z) { mNormal = Eigen::Vector3f(x,y,z); }
  void setPosition(const Eigen::Vector3f& pos) { mPosition = pos; }
  void setPosition(float x, float y, float z) { mPosition = Eigen::Vector3f(x,y,z); }

  const Eigen::Vector3f& position() const { return mPosition; }
  const Eigen::Vector3f& normal() const { return mNormal; }

  virtual ~Surflet()  {  }

//  friend std::ostream& operator<<(std::ostream& stream, const sure::Surflet& rhs);


private:
  Eigen::Vector3f mNormal;
  Eigen::Vector3f mPosition;

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar & mNormal[0];
    ar & mNormal[1];
    ar & mNormal[2];
    ar & mPosition[0];
    ar & mPosition[1];
    ar & mPosition[2];
  }

};


void calculateSurfletPairRelations(const sure::Surflet& p1, const sure::Surflet& p2, double& alpha, double& phi, double& theta);

//std::ostream& operator<<(std::ostream& stream, const sure::Surflet& rhs);

}

#endif /* SURFLET_H_ */
