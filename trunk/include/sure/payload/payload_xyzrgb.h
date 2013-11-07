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

#ifndef SURE_PAYLOAD_XYZRGB_H_
#define SURE_PAYLOAD_XYZRGB_H_

#include <ostream>
#include <iomanip>
#include <Eigen/Core>

#include <sure/normal/normal.h>
#include <sure/payload/payload.h>


namespace sure
{
  namespace payload
  {
    typedef sure::payload::Payload Payload;
    typedef sure::PointFlag PointFlag;

    /**
     * Payload for integrating point and color information during octree creation
     */
    class PointsRGB : public Payload
    {
      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PointsRGB() : Payload(), colorSum_(Vector3::Zero()),
        pointSqrSum_(Matrix3::Zero()), pointSum_(Vector3::Zero()), pointMean_(Vector3::Zero()),
        points_(0), flag_(NO_FLAG) { }

        virtual ~PointsRGB() { }

        PointsRGB operator+(const PointsRGB& rhs) const
        {
          PointsRGB r(*this);

          r.colorSum_ += rhs.colorSum_;
          r.pointSqrSum_ += rhs.pointSqrSum_;
          r.pointSum_ += rhs.pointSum_;
          r.points_ += rhs.points_;
          r.pointMean_ = r.pointSum_ / (Scalar) r.points_;
          r.flag_ = (PointFlag) ((PointFlag) r.flag_ | (PointFlag) rhs.flag_);
          return r;
        }

        PointsRGB& operator+=(const PointsRGB& rhs)
        {
          this->points_ += rhs.points_;
          this->colorSum_ += rhs.colorSum_;
          this->pointSqrSum_ += rhs.pointSqrSum_;
          this->pointSum_ += rhs.pointSum_;
          this->pointMean_ = this->pointSum_ / (Scalar) this->points_;
          this->flag_ = (PointFlag) ((PointFlag) this->flag_ | (PointFlag) rhs.flag_);
          return *this;
        }

        void clear()
        {
          colorSum_ = pointSum_ = Vector3::Zero();
          pointSqrSum_ = Matrix3::Zero();
          points_ = 0;
          flag_ = NORMAL;
        }

        const Vector3& getPosSum() const { return pointSum_; }
        const Matrix3& getPosSqrSum() const { return pointSqrSum_; }
        const Vector3& getColorSum() const { return colorSum_; }
        unsigned getPointCount() const { return points_; }
        PointFlag getPointFlag() const { return flag_; }

        Vector3 getMeanPosition() const { return pointMean_; }
        Vector3 getMeanColor() const { return colorSum_ / (Scalar) points_; }
        Scalar red() const { return colorSum_[0] / (Scalar) points_; }
        Scalar green() const { return colorSum_[1] / (Scalar) points_; }
        Scalar blue() const { return colorSum_[2] / (Scalar) points_; }

        void setPosition(const Vector3& xyz)
        {
          this->setPosition(xyz[0], xyz[1], xyz[2]);
        }
        void setPosition(Scalar x, Scalar y, Scalar z)
        {
          this->pointSum_ = this->pointMean_ = Vector3(x, y, z);
          this->pointSqrSum_ = pointSum_ * pointSum_.transpose();
          points_ = 1;
        }

        void setColor(const Vector3& rgb)
        {
          this->setColor(rgb[0], rgb[1], rgb[2]);
        }
        void setColor(Scalar r, Scalar g, Scalar b)
        {
          this->colorSum_ = Vector3(r, g, b);
        }
        void setColor(float rgb)
        {
          int color = *reinterpret_cast<const int*>(&rgb);
          float r = float((0xff0000 & color) >> 16) / 255.f;
          float g = float((0x00ff00 & color) >> 8) / 255.f;
          float b = float( 0x0000ff & color) / 255.f;
          this->setColor(r, g, b);
        }

        void setFlag(PointFlag flag) { flag_ = flag; }

        bool valid() const
        {
          return eigen_is_finite(colorSum_) && eigen_is_finite(pointSqrSum_) && eigen_is_finite(pointSum_);
        }

        /**
         * Calculates the normal using the integrated point data
         */
        sure::normal::Normal calculateNormal() const;

      protected:

        Vector3 colorSum_;
        Matrix3 pointSqrSum_;
        Vector3 pointSum_;
        Vector3 pointMean_;
        unsigned points_;
        PointFlag flag_;


      private:

        friend std::ostream& operator<<(std::ostream& stream, PointsRGB& p);

    };

    std::ostream& operator<<(std::ostream& stream, PointsRGB& rhs);

  }
}




#endif /* SURE_PAYLOAD_XYZRGB_H_ */
