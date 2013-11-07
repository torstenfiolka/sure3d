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

#include <pcl/common/eigen.h>

#include <sure/payload/payload_xyzrgb.h>

std::ostream& sure::payload::operator<<(std::ostream& stream, PointsRGB& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(2);
  stream << "Payload XYZRGB - number of points: " << rhs.points_ << " - flags: ";
  stream << (((rhs.getPointFlag() & NORMAL) == NORMAL) ? "normal " : "")
         << (((rhs.getPointFlag() & ARTIFICIAL) == ARTIFICIAL) ? "artificial " : "")
         << (((rhs.getPointFlag() & BACKGROUND_BORDER) == BACKGROUND_BORDER) ? "background " : "")
         << (((rhs.getPointFlag() & FOREGROUND_BORDER) == FOREGROUND_BORDER) ? "foreground\n" : "\n");
  stream << "xyz: " << rhs.pointSum_[0] << "/" << rhs.pointSum_[1] << "/" << rhs.pointSum_[2] <<  stream << " - rgb: " << rhs.colorSum_[0] << "/" << rhs.colorSum_[1] << "/" << rhs.colorSum_[2] << "\n";
  stream << "xyz^2: " << rhs.pointSqrSum_.row(0)[0] << "/" << rhs.pointSqrSum_.row(0)[1] << "/" << rhs.pointSqrSum_.row(0)[2] << " - "
                      << rhs.pointSqrSum_.row(1)[0] << "/" << rhs.pointSqrSum_.row(1)[1] << "/" << rhs.pointSqrSum_.row(1)[2] << " - "
                      << rhs.pointSqrSum_.row(2)[0] << "/" << rhs.pointSqrSum_.row(2)[1] << "/" << rhs.pointSqrSum_.row(2)[2] << "\n";
  stream.unsetf(std::ios_base::fixed);
  return stream;
}

sure::normal::Normal sure::payload::PointsRGB::calculateNormal() const
{
  sure::normal::Normal normal;
  if( points_ < sure::normal::MIN_NUMBER_OF_POINTS_FOR_STABLE_NORMAL )
  {
    normal.setUnstable();
    return normal;
  }

  Matrix3 summedSquares(pointSqrSum_ / (sure::Scalar) points_);
  Vector3 summedPosition(pointSum_ / (sure::Scalar) points_);

  summedSquares -= summedPosition * summedPosition.transpose();

  if( eigen_is_finite(summedSquares) )
  {
    sure::Scalar eigenValue;
    pcl::eigen33(summedSquares, eigenValue, normal.vector());

    if( eigen_is_finite(normal.vector()) )
    {
      normal.setStable();
    }
  }
  return normal;
}
