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

#ifndef SURE_NORMAL_H_
#define SURE_NORMAL_H_

#include <ostream>
#include <iomanip>
#include <Eigen/Dense>

#include <sure/data/typedef.h>

namespace sure
{
  namespace normal
  {

    class Normal
    {
      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum NormalStatus
        {
          NORMAL_NOT_CALCULATED = 0,
          NORMAL_STABLE,
          NORMAL_UNSTABLE
        };

        Normal() : n(NormalType::Zero()), status(NORMAL_NOT_CALCULATED) { }

        virtual ~Normal() { }

        Normal& operator=(const NormalType& rhs) { this->n = rhs; return *this; }

        const NormalType& vector() const { return n; }
        NormalType& vector() { return n; }

        Scalar& operator[](unsigned int index) { return n[index]; }
        Scalar operator[](unsigned int index) const { return n[index]; }

        void setStable() { status = NORMAL_STABLE; }
        void setUnstable() { status = NORMAL_UNSTABLE; }
        NormalStatus getStatus() const { return status; }
        bool isStable() const { return status == NORMAL_STABLE; }

        NormalType getVector() const { return n; }


      protected:

        NormalType n;
        NormalStatus status;

        friend std::ostream& operator<<(std::ostream& stream, const Normal& normal);
    };

    std::ostream& operator<<(std::ostream& stream, const Normal& normal);

  } // namespace
} // namespace

#endif /* SURE_NORMAL_H_ */
