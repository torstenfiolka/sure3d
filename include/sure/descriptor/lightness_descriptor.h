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

#ifndef SURE_LIGHTNESS_DESCRIPTOR_H_
#define SURE_LIGHTNESS_DESCRIPTOR_H_

#include <sure/descriptor/histogram.h>

namespace sure
{
  namespace descriptor
  {

    typedef sure::descriptor::Histogram<LIGHTNESS_DESCRIPTOR_SIZE> LightnessHistogram;

    class LightnessDescriptor : public LightnessHistogram
    {
      public:

        LightnessDescriptor() : LightnessHistogram(-1.0, 1.0), referenceLightness_(0.0) { }

        LightnessDescriptor(const LightnessDescriptor& rhs) : LightnessHistogram(rhs), referenceLightness_(rhs.referenceLightness_) { }

        void setLightness(HistoType lightness) { this->referenceLightness_ = lightness; }
        HistoType getLightness() const { return referenceLightness_; }

        void insertValue(HistoType lightness) { LightnessHistogram::insertSmooth(lightness - referenceLightness_, 1.0, false); }
        Scalar distanceTo(const LightnessDescriptor& rhs) const { return LightnessHistogram::L2Distance((LightnessHistogram) rhs); }

      protected:

        HistoType referenceLightness_;

      private:

        friend std::ostream& operator<<(std::ostream& stream, const LightnessDescriptor& rhs);
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & (LightnessHistogram) *this;
            ar & referenceLightness_;
        }

    };

    std::ostream& operator<<(std::ostream& stream, const LightnessDescriptor& rhs);


  } // namespace
} // namespace

#endif /* SURE_LIGHTNESS_DESCRIPTOR_H_ */
