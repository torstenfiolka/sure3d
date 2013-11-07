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

#ifndef CROSS_PRODUCT_HISTOGRAM_H_
#define CROSS_PRODUCT_HISTOGRAM_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_types.h>

#include <sure/normal/base_histogram.h>

namespace sure
{

  namespace normal
  {

    //! stores the histogram for a normal
    class CrossProductHistogram : public BaseHistogram<CROSS_PRODUCT_HISTOGRAM_SIZE>
    {
      public:

        CrossProductHistogram() : BaseHistogram<CROSS_PRODUCT_HISTOGRAM_SIZE>()
        {
        }

        virtual ~CrossProductHistogram() {}

        void insertCrossProduct(const NormalType& referenceNormal, const NormalType& secondNormal);

        HistoType calculateEntropy() const { return calculateRawEntropy() / MAX_ENTROPY; }

      protected:

        static const int CODIRECTION_CROSSPRODUCT_BIN = CROSS_PRODUCT_HISTOGRAM_SIZE-1;
        static const HistoType MAX_ENTROPY;

    };

  } // namespace

} // namespace

#endif /* CROSS_PRODUCT_HISTOGRAM_H_ */
