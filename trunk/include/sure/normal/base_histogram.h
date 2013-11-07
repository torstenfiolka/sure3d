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

#ifndef BASE_HISTOGRAM_H_
#define BASE_HISTOGRAM_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <pcl/point_types.h>

#include <sure/data/typedef.h>

namespace sure
{

  namespace normal
  {

    template <int HistogramSize>
    class BaseHistogram;

    template <int HistogramSize>
    std::ostream& operator<<(std::ostream& stream, const BaseHistogram<HistogramSize>& rhs);

    //! stores the histogram for a normal
    template <int HistogramSize>
    class BaseHistogram
    {
      public:

        static const HistoType MAX_ENTROPY;

        BaseHistogram() : numberOfEntries_(0), weight_(0.0), influenceRadius_(0.0)
        {
          clear();
        }

        virtual ~BaseHistogram() {}

        //! sets the + operator for adding histograms
        BaseHistogram operator+(const BaseHistogram& rhs) const
        {
          BaseHistogram h = *this;
          h.numberOfEntries_ += rhs.numberOfEntries_;
          h.weight_ += rhs.weight_;
          for(int i=0; i<NORMAL_HISTOGRAM_SIZE; ++i)
          {
            h.values_[i] += rhs.values_[i];
          }
          return h;
        }
        BaseHistogram& operator+=(const BaseHistogram& rhs)
        {
          this->weight_ += rhs.weight_;
          this->numberOfEntries_ += rhs.numberOfEntries_;
          for(int i=0; i<NORMAL_HISTOGRAM_SIZE; ++i)
          {
            this->values_[i] += rhs.values_[i];
          }
          return *this;
        }

        //! sets the * operator for normalizing/scaling the histogram
        BaseHistogram operator*(const HistoType& rhs) const
        {
          BaseHistogram h = *this;
          h.weight_ *= rhs;
          for(int i=0; i<NORMAL_HISTOGRAM_SIZE; ++i)
          {
            h.values_[i] *= rhs;
          }
          return h;
        }
        BaseHistogram& operator*=(const HistoType& rhs)
        {
          this->weight_ *= rhs;
          for(int i=0; i<NORMAL_HISTOGRAM_SIZE; ++i)
          {
            this->values_[i] *= rhs;
          }
          return *this;
        }

        //! clears all data
        void clear();

        void setInfluenceRadius(Scalar r) { this->influenceRadius_ = cos(r); }
        Scalar getInfluenceRadius() const { return acos(this->influenceRadius_); }

        //! returns a pcl::histogram
        pcl::Histogram<HistogramSize> getHistogram() const;

      protected:

        //! calculates the raw entropy of the histogram
        HistoType calculateRawEntropy() const;

        //! the histogram
        HistoType values_[HistogramSize];

        unsigned numberOfEntries_;
        HistoType weight_;

        Scalar influenceRadius_;

        friend std::ostream& operator<< <>(std::ostream& stream, const BaseHistogram& rhs);

    };

    //! Initializes the histogram classes
    std::vector<NormalType, NormalTypeAllocator> initVectors(unsigned numberOfPolarBins);

    const std::vector<NormalType, NormalTypeAllocator> REFERENCE_VECTORS = initVectors(NORMAL_HISTOGRAM_POLAR_SPLITS);

  } // namespace

} // namespace

#include <sure/normal/impl/base_histogram.hpp>

#endif /* BASE_HISTOGRAM_H_ */
