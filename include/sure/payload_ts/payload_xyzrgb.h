/*
 * payload_xyzrgb.h
 *
 *  Created on: 02.08.2013
 *      Author: fiolka
 */

#ifndef SURE_PAYLOAD_XYZRGB_H_
#define SURE_PAYLOAD_XYZRGB_H_

#include <ostream>
#include <Eigen/Core>

#include <sure/payload/payload.h>

namespace sure
{
  namespace payload
  {
    using sure::payload::Payload;
    typedef Eigen::Vector3d Vector3;
    typedef Eigen::Matrix3d Matrix3;

    class PointsRGB : public Payload
    {
      public:

        PointsRGB() : Payload(), colorSum_(Vector3::Zero()),
          pointSqrSum_(Matrix3::Zero()), pointSum_(Vector3::Zero()), points_(0) { }

        PointsRGB(const PointsRGB& rhs) : Payload()
        {
          this->colorSum_ = rhs.colorSum_;
          this->pointSqrSum_ = rhs.pointSqrSum_;
          this->pointSum_ = rhs.pointSum_;
          this->points_ = rhs.points_;
        }
        virtual ~PointsRGB() { }

//        Payload_XYZRGB operator+(const Payload_XYZRGB& rhs)
//        {
//          shared_lock sl(this->mutex_);
//          lock_guard lg(this->mutex_, adopt_lock);
//          Payload_XYZRGB r(*this);
//
//          r.rgbSum_ += rhs.rgbSum_;
//          r.squaredSum_ += rhs.squaredSum_;
//          r.xyzsum_ += rhs.xyzsum_;
//          return r;
//        }

        PointsRGB& operator+=(const PointsRGB& rhs)
        {
          unique_lock ul(this->mutex_);
          lock_guard lug(this->mutex_, adopt_lock);

          this->points_ += rhs.points_;
          this->colorSum_ += rhs.colorSum_;
          this->pointSqrSum_ += rhs.pointSqrSum_;
          this->pointSum_ += rhs.pointSum_;
          ul.unlock();
          return *this;
        }

        PointsRGB& operator=(const PointsRGB& rhs)
        {
          unique_lock ul(this->mutex_);
          lock_guard lg(this->mutex_, adopt_lock);
          if( this != &rhs )
          {
            this->colorSum_ = rhs.colorSum_;
            this->pointSqrSum_ = rhs.pointSqrSum_;
            this->pointSum_ = rhs.pointSum_;
            this->points_ = rhs.points_;
          }
          return *this;
        }

//        Payload_XYZRGB operator=(Payload_XYZRGB& rhs) const
//        {
//          Payload_XYZRGB r(rhs);
//          return r;
//        }

        void clear()
        {
          unique_lock ul(this->mutex_);
          lock_guard lg(this->mutex_, adopt_lock);
          colorSum_ = pointSum_ = Vector3::Zero();
          pointSqrSum_ = Matrix3::Zero();
          points_ = 0;
        }

        void setPosition(const Vector3& xyz)
        {
          this->setPosition(xyz[0], xyz[1], xyz[2]);
        }

        void setPosition(float x, float y, float z)
        {
          unique_lock ul(this->mutex_);
          lock_guard lg(this->mutex_, adopt_lock);
          this->pointSum_ = Vector3(x, y, z);
          this->pointSqrSum_ = pointSum_ * pointSum_.transpose();
          points_ = 1;
        }

        void setColor(const Vector3& rgb)
        {
          this->setColor(rgb[0], rgb[1], rgb[2]);
        }

        void setColor(float r, float g, float b)
        {
          unique_lock ul(this->mutex_);
          lock_guard lg(this->mutex_, adopt_lock);
          this->colorSum_ = Vector3(r, g, b);
        }


      protected:

        Vector3 colorSum_;
        Matrix3 pointSqrSum_;
        Vector3 pointSum_;
        unsigned points_;

      private:

        friend std::ostream& operator<<(std::ostream& stream, PointsRGB& p);

    };

    std::ostream& operator<<(std::ostream& stream, PointsRGB& rhs)
    {
      shared_lock sl(rhs.mutex_);
      lock_guard lg(rhs.mutex_, adopt_lock);
      stream.setf(std::ios_base::fixed);
      stream << std::setprecision(2);
      stream << "XYZ#: " << rhs.pointSum_[0] << "/" << rhs.pointSum_[1] << "/" << rhs.pointSum_[2] << " / " << rhs.points_ << "\n";
      stream << "RGB: " << rhs.colorSum_[0] << "/" << rhs.colorSum_[1] << "/" << rhs.colorSum_[2] << "\n";
      stream << "XYZ*XYZ: " << rhs.pointSqrSum_.row(0)[0] << "/" << rhs.pointSqrSum_.row(0)[1] << "/" << rhs.pointSqrSum_.row(0)[2] << "\n";
      stream << "XYZ*XYZ: " << rhs.pointSqrSum_.row(1)[0] << "/" << rhs.pointSqrSum_.row(1)[1] << "/" << rhs.pointSqrSum_.row(1)[2] << "\n";
      stream << "XYZ*XYZ: " << rhs.pointSqrSum_.row(2)[0] << "/" << rhs.pointSqrSum_.row(2)[1] << "/" << rhs.pointSqrSum_.row(2)[2] << "\n";
      stream.unsetf(std::ios_base::fixed);
      return stream;
    }

  }
}




#endif /* SURE_PAYLOAD_XYZRGB_H_ */
