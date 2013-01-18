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

#ifndef SURFLET_H_
#define SURFLET_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <iostream>
#include <iomanip>

namespace sure
{

class Surflet
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f normal;
  Eigen::Vector3f point;

  Surflet()
  {
    this->reset();
  }

  Surflet(const Eigen::Vector3f pos)
  {
    this->reset();
    this->point = pos;
  }

  Surflet(const Eigen::Vector3f pos, const Eigen::Vector3f normal)
  {
    this->point = pos;
    this->normal = normal;
  }

  virtual void reset()
  {
    normal = Eigen::Vector3f::Zero();
    point = Eigen::Vector3f::Zero();
  }

  virtual void print() const
  {
    std::cout << std::fixed << std::setprecision(3) << "[Surflet] Position: " << point[0] << " / " << point[1] << " / " << point[2] << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "[Surflet] Normal: " << normal[0] << " / " << normal[1] << " / " << normal[2] << std::endl;
  }

  virtual ~Surflet()  {  }

  friend std::ostream& operator<<(std::ostream& stream, const sure::Surflet& rhs);
};

void calculateSurfletPairRelations(const sure::Surflet& p1, const sure::Surflet& p2, double& alpha, double& phi, double& theta);

std::ostream& operator<<(std::ostream& stream, const sure::Surflet& rhs);

}

#endif /* SURFLET_H_ */
