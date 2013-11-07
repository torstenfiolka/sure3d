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

#ifndef SURE_PAYLOAD_CROSS_PRODUCT_H_
#define SURE_PAYLOAD_CROSS_PRODUCT_H_

#include <sure/normal/normal.h>
#include <sure/normal/cross_product_histogram.h>
#include <sure/payload/payload.h>

namespace sure
{
  namespace payload
  {

    /*
     * Optional payload for storing a normal and its histogram
     * + and += may be used to accumulate the histogram
     */
    class CrossProductPayload : public sure::payload::Payload
    {
      public:

        // Stores the normal
        sure::normal::Normal normal_;

        // Stores the discretized normal
        sure::normal::CrossProductHistogram histogram_;

        CrossProductPayload operator+(const CrossProductPayload& rhs) const
        {
          CrossProductPayload r(*this);
          r.histogram_ += rhs.histogram_;
          return r;
        }

        CrossProductPayload& operator+=(const CrossProductPayload& rhs)
        {
          this->histogram_ += rhs.histogram_;
          return *this;
        }

      protected:

        friend std::ostream& operator<<(std::ostream& stream, const CrossProductPayload& rhs);
    };

    std::ostream& operator<<(std::ostream& stream, const CrossProductPayload& rhs);

  }
}


#endif /* SURE_PAYLOAD_CROSS_PRODUCT_H_ */
