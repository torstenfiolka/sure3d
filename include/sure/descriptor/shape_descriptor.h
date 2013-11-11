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

#ifndef SURE_SHAPE_DESCRIPTOR_H_
#define SURE_SHAPE_DESCRIPTOR_H_

#include <cmath>
#include <sure/descriptor/histogram.h>

namespace sure
{
  namespace descriptor
  {

    typedef sure::descriptor::Histogram<SHAPE_DESCRIPTOR_SIZE> ShapeHistogram;

    void calculateSurfletPairRelations(const Vector3& pos1, const Vector3& normal1, const Vector3& pos2, const Vector3& normal2, HistoType& alpha, HistoType& phi, HistoType& theta);

    class ShapeDescriptor
    {
      public:

        ShapeDescriptor() : alpha_(ALPHA_MIN, ALPHA_MAX), phi_(PHI_MIN, PHI_MAX), theta_(THETA_MIN, THETA_MAX) { }

        ShapeDescriptor(const ShapeDescriptor& rhs) : alpha_(rhs.alpha_), phi_(rhs.phi_), theta_(rhs.theta_) { }

        void clear();

        ShapeDescriptor operator +(const ShapeDescriptor& rhs) const
        {
          ShapeDescriptor h(*this);
          h.alpha_ += rhs.alpha_;
          h.phi_ += rhs.phi_;
          h.theta_ += rhs.theta_;
          return h;
        }
        ShapeDescriptor& operator +=(const ShapeDescriptor& rhs)
        {
          this->alpha_ += rhs.alpha_;
          this->phi_ += rhs.phi_;
          this->theta_ += rhs.theta_;
          return *this;
        }

        ShapeDescriptor operator *(HistoType rhs) const
        {
          ShapeDescriptor h(*this);
          h.alpha_ *= rhs;
          h.phi_ *= rhs;
          h.theta_ *= rhs;
          return h;
        }
        ShapeDescriptor& operator *=(HistoType rhs)
        {
          this->alpha_ *= rhs;
          this->phi_ *= rhs;
          this->theta_ *= rhs;
          return *this;
        }

        ShapeDescriptor operator /(HistoType rhs) const
        {
          ShapeDescriptor h(*this);
          h.alpha_ /= rhs;
          h.phi_ /= rhs;
          h.theta_ /= rhs;
          return h;
        }
        ShapeDescriptor& operator /=(HistoType rhs)
        {
          this->alpha_ /= rhs;
          this->phi_ /= rhs;
          this->theta_ /= rhs;
          return *this;
        }

        void normalize();
        bool isNormalized() const
        {
          return alpha_.isNormalized() && phi_.isNormalized() && theta_.isNormalized();
        }

        void insertValues(HistoType alpha, HistoType phi, HistoType theta);
        Scalar distanceTo(const ShapeDescriptor& rhs) const;

      protected:

        ShapeHistogram alpha_;
        ShapeHistogram phi_;
        ShapeHistogram theta_;

      private:

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & alpha_;
            ar & phi_;
            ar & theta_;
        }

        friend std::ostream& operator<<(std::ostream& stream, const ShapeDescriptor& rhs);
    };

    std::ostream& operator<<(std::ostream& stream, const ShapeDescriptor& rhs);

  } // namespace
} // namespace

#endif /* SURE_SHAPE_DESCRIPTOR_H_ */
