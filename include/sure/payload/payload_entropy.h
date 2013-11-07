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

#ifndef PAYLOAD_ENTROPY_H_
#define PAYLOAD_ENTROPY_H_

#include <sure/data/typedef.h>
#include <sure/payload/payload.h>
#include <iomanip>

namespace sure
{
  namespace payload
  {
    typedef sure::payload::Payload Payload;
    typedef sure::MaximumFlag MaximumFlag;

    /**
     * Optional payload for storing necessary variable for the entropy calculation
     */
    class EntropyPayload : public Payload
    {
      public:

        EntropyPayload() : entropy_(0.0), cornerness_(0.0), flag_(NOT_CALCULATED) { }
        virtual ~EntropyPayload() { }

        // Stores the calculated entropy
        Scalar entropy_;

        // Stores the calculated cornerness
        Scalar cornerness_;

        /**
         * Stores the current status of a node concerning entropy calculation.
         * Nodes inept for a feature will be flagged appropriate and skipped in following calculation steps
         */
        MaximumFlag flag_;

      protected:

        friend std::ostream& operator<<(std::ostream& stream, const EntropyPayload& rhs);

    };

    std::ostream& operator<<(std::ostream& stream, const EntropyPayload& rhs);

  }
}


#endif /* PAYLOAD_ENTROPY_H_ */
