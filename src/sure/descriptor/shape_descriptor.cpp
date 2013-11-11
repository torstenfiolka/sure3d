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

#include <sure/descriptor/shape_descriptor.h>

void sure::descriptor::calculateSurfletPairRelations(const Vector3& pos1, const Vector3& normal1, const Vector3& pos2, const Vector3& normal2, HistoType& alpha, HistoType& phi, HistoType& theta)
{
  Vector3 v, w, p2p1;
  p2p1 = Eigen::Vector3d(pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]);

  const Vector3& u = normal1;
  v = u.cross(p2p1);
  v.normalize();
  w = v.cross(u);
  w.normalize();
  alpha = (HistoType) v.dot(normal2);
  phi = (HistoType) ( u.dot(p2p1) ) / p2p1.norm();
  theta = (HistoType) atan2(w.dot(normal2), u.dot(normal2));
}

void sure::descriptor::ShapeDescriptor::clear()
{
  alpha_.clear();
  phi_.clear();
  theta_.clear();
}

void sure::descriptor::ShapeDescriptor::normalize()
{
  alpha_.normalize();
  phi_.normalize();
  theta_.normalize();
}

sure::Scalar sure::descriptor::ShapeDescriptor::distanceTo(const ShapeDescriptor& rhs) const
{
  Scalar distance(0.0);
  distance += alpha_.L2Distance(rhs.alpha_);
  distance += phi_.L2Distance(rhs.phi_);
  distance += theta_.L2Distance(rhs.theta_);
  return (distance / 3.0);
}

void sure::descriptor::ShapeDescriptor::insertValues(HistoType alpha, HistoType phi, HistoType theta)
{
  alpha_.insertSmooth(alpha, SHAPE_HISTOGRAM_VALUE, false);
  phi_.insertSmooth(phi, SHAPE_HISTOGRAM_VALUE, false);
  theta_.insertSmooth(theta, SHAPE_HISTOGRAM_VALUE, false);
}

std::ostream& sure::descriptor::operator<<(std::ostream& stream, const ShapeDescriptor& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(2);
  stream << "Shape - Alpha " << rhs.alpha_;
  stream << "Phi " << rhs.phi_;
  stream << "Theta " << rhs.theta_;
  stream.unsetf(std::ios_base::fixed);
  return stream;
}

BOOST_CLASS_VERSION(sure::descriptor::ShapeHistogram, 0)
BOOST_CLASS_VERSION(sure::descriptor::ShapeDescriptor, 0)
