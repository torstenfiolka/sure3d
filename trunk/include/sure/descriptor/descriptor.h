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

#ifndef SURE_DESCRIPTOR_H_
#define SURE_DESCRIPTOR_H_

#include <cmath>
#include <Eigen/Dense>

#include <sure/descriptor/color_descriptor.h>
#include <sure/descriptor/lightness_descriptor.h>
#include <sure/descriptor/shape_descriptor.h>

namespace sure
{
  namespace descriptor
  {

    class Descriptor
    {
      public:

        Descriptor() { }

        Descriptor operator+(const Descriptor& rhs) const
        {
          Descriptor lhs(*this);
          lhs.shape_ += rhs.shape_;
          lhs.color_ += rhs.color_;
          lhs.lightness_ += rhs.lightness_;
          return lhs;
        }
        Descriptor& operator+=(const Descriptor& rhs)
        {
          this->shape_ += rhs.shape_;
          this->color_ += rhs.color_;
          this->lightness_ += rhs.lightness_;
          return *this;
        }

        Descriptor operator*(HistoType rhs) const
        {
          Descriptor lhs(*this);
          lhs.shape_ *= rhs;
          lhs.color_ *= rhs;
          lhs.lightness_ *= rhs;
          return lhs;
        }
        Descriptor& operator*=(HistoType rhs)
        {
          this->shape_ *= rhs;
          this->color_ *= rhs;
          this->lightness_ *= rhs;
          return *this;
        }

        Descriptor operator/(HistoType rhs) const
        {
          Descriptor lhs(*this);
          lhs.shape_ /= rhs;
          lhs.color_ /= rhs;
          lhs.lightness_ /= rhs;
          return lhs;
        }
        Descriptor& operator/=(HistoType rhs)
        {
          this->shape_ /= rhs;
          this->color_ /= rhs;
          this->lightness_ /= rhs;
          return *this;
        }

        ~Descriptor() {}

        const ShapeDescriptor& shape() const { return shape_; }
        ShapeDescriptor& shape() { return shape_; }

        const ColorDescriptor& color() const { return color_; }
        ColorDescriptor& color() { return color_; }

        const LightnessDescriptor& lightness() const { return lightness_; }
        LightnessDescriptor& lightness() { return lightness_; }

        void clear();

        bool isNormalized() const;
        void normalize();

        Scalar distanceTo(const Descriptor& rhs, Scalar shapeWeight = 1.0, Scalar colorWeight = 1.0, Scalar lightnessWeight = 1.0) const;

        void createRandomDescriptor();

      protected:

        ShapeDescriptor shape_;
        ColorDescriptor color_;
        LightnessDescriptor lightness_;

      private:

        friend std::ostream& operator<<(std::ostream& stream, const Descriptor& rhs);

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & shape_;
            ar & color_;
            ar & lightness_;
        }

    };

    std::ostream& operator<<(std::ostream& stream, const Descriptor& rhs);

  } // namespace
} // namespace

#endif /* FEATURE_H_ */
