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

#include "sure/surflet.h"

//! calculates surflet pair relations for two given suflets
void sure::calculateSurfletPairRelations(const sure::Surflet& reference, const sure::Surflet& neighbour, double& alpha, double& phi, double& theta)
{
  Eigen::Vector3d u, v, w, p2p1;
  p2p1 = Eigen::Vector3d(neighbour.point[0] - reference.point[0], neighbour.point[1] - reference.point[1], neighbour.point[2] - reference.point[2]);
  u = Eigen::Vector3d(reference.normal[0], reference.normal[1], reference.normal[2]);
  u.normalize();
  v = u.cross(p2p1);
  v.normalize();
  w = v.cross(u);
  w.normalize();
  alpha = (double) v.dot(Eigen::Vector3d(neighbour.normal[0], neighbour.normal[1], neighbour.normal[2]));
  phi = (double) (u.dot(p2p1))/p2p1.norm();
  theta = (double) atan2(w.dot(Eigen::Vector3d(neighbour.normal[0], neighbour.normal[1], neighbour.normal[2])), u.dot(Eigen::Vector3d(neighbour.normal[0], neighbour.normal[1], neighbour.normal[2])));
}

//std::ostream& sure::operator<<(std::ostream& stream, const sure::Surflet& rhs)
//{
//  stream << std::fixed << std::setprecision(3) << "[Surflet] Position: " << rhs.point[0] << " / " << rhs.point[1] << " / " << rhs.point[2] << std::endl;
//  stream << std::fixed << std::setprecision(3) << "[Surflet] Normal: " << rhs.normal[0] << " / " << rhs.normal[1] << " / " << rhs.normal[2] << std::endl;
//  return stream;
//}

BOOST_CLASS_VERSION(sure::Surflet, 0)
