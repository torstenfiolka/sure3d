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

#ifndef SURE_HISTOGRAM_H_
#define SURE_HISTOGRAM_H_

#include <vector>
#include <iostream>
#include <iomanip>

#include <pcl/common/common.h>
#include <pcl/common/norms.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/base_object.hpp>

#include <FastEMD/emd_hat.hpp>

#include <sure/data/typedef.h>


namespace sure
{
  namespace descriptor
  {

    template <int SizeT>
    class Histogram;

    template <int SizeT>
    std::ostream& operator<<(std::ostream& stream, const Histogram<SizeT>& rhs);

    /**
     * Base class for descriptors
     */
    template <int SizeT>
    class Histogram
    {
      public:

        Histogram(HistoType min = 0.0, HistoType max = 1.0) : min_(min), max_(max), binSize_((max_-min_) / (HistoType) SizeT) { clear(); }

        Histogram(const Histogram& rhs) : min_(rhs.min_), max_(rhs.max_), binSize_(rhs.binSize_), weight_(rhs.weight_)
        {
          for(unsigned i=0; i<SizeT; ++i)
          {
            array_[i] = rhs.array_[i];
          }
        }

        virtual ~Histogram() {}

        virtual void clear()
        {
          for(unsigned i=0; i<SizeT; ++i)
          {
            array_[i] = 0.0;
          }
          weight_ = 0.0;
        }

        Histogram operator +(const Histogram& rhs) const
        {
          Histogram h(*this);
          for(unsigned i=0; i<SizeT; ++i)
          {
            h.array_[i] += rhs.array_[i];
          }
          h.weight_ += rhs.weight_;
          return h;
        }

        Histogram& operator +=(const Histogram& rhs)
        {
          for(unsigned i=0; i<SizeT; ++i)
          {
            this->array_[i] += rhs.array_[i];
          }
          this->weight_ += rhs.weight_;
          return *this;
        }

        Histogram operator *(HistoType rhs) const
        {
          Histogram h(*this);
          for(unsigned i=0; i<SizeT; ++i)
          {
            h.array_[i] *= rhs;
          }
          h.weight_ *= rhs;
          return h;
        }

        Histogram& operator *=(HistoType rhs)
        {
          for(unsigned i=0; i<SizeT; ++i)
          {
            this->array_[i] *= rhs;
          }
          this->weight_ *= rhs;
          return *this;
        }

        Histogram operator /(HistoType rhs) const
        {
          Histogram h(*this);
          for(unsigned i=0; i<SizeT; ++i)
          {
            h.array_[i] /= rhs;
          }
          h.weight_ /= rhs;
          return h;
        }

        Histogram& operator /=(HistoType rhs)
        {
          for(unsigned i=0; i<SizeT; ++i)
          {
            this->array_[i] /= rhs;
          }
          this->weight_ /= rhs;
          return *this;
        }

        const HistoType& operator[](unsigned index) const { return array_[index]; }

        virtual void normalize()
        {
          if( weight_ > 0.0 )
          {
            *this /= weight_;
          }
          weight_ = 1.0;
        }
        bool isNormalized() const { return (weight_ == 1.0); }
        HistoType weight() const { return weight_; }
        int size() const { return SizeT; }

      protected:

        void insertSmooth(HistoType value, HistoType weight = 1.0, bool circumferential = false);
        void insertPrecise(HistoType value, HistoType weight = 1.0);

        void fillHistogramRandom(int seed = 0);

        HistoType EMDistance(const Histogram& rhs, const std::vector<std::vector<double> >& distanceMatrix) const;
        HistoType L2Distance(const Histogram& rhs) const;

        void fillVector(std::vector<double>& vec) const;

        HistoType array_[SizeT];
        HistoType min_;
        HistoType max_;
        HistoType binSize_;
        HistoType weight_;

      private:

        friend std::ostream& operator<< <>(std::ostream& stream, const Histogram<SizeT>& rhs);
        friend class boost::serialization::access;
        friend class ShapeDescriptor;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & min_;
            ar & max_;
            for(int i=0; i<SizeT; ++i)
            {
              ar & array_[i];
            }
            ar & binSize_;
            ar & weight_;
        }

    };

    template <int SizeT>
    std::vector<std::vector<double> > initEMDMatrix(double maxDistance);

  } // namespace
} // namespace

#include <sure/descriptor/impl/histogram.hpp>

#endif /* SURE_HISTOGRAM_H_ */
