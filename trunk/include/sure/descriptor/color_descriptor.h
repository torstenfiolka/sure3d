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

#ifndef SURE_COLOR_DESCRIPTOR_H_
#define SURE_COLOR_DESCRIPTOR_H_

#include <sure/descriptor/histogram.h>

namespace sure
{
  namespace descriptor
  {

    typedef sure::descriptor::Histogram<COLOR_DESCRIPTOR_SIZE> ColorHistogram;

    std::vector<std::vector<double> > initColorDescriptorEMDMatrix();

    void convertRGBtoHSL(Scalar r, Scalar g, Scalar b, Scalar& h, Scalar& s, Scalar& l);

    class ColorDescriptor : public ColorHistogram
    {
      public:

        ColorDescriptor() : ColorHistogram(MIN_HUE, MAX_HUE), saturationBalance_(0.0) { }

        ColorDescriptor(const ColorDescriptor& rhs) : ColorHistogram(rhs), saturationBalance_(rhs.saturationBalance_) { }

        ~ColorDescriptor() { }

        virtual void clear();
        virtual void normalize();

        void insertValue(Scalar hue, Scalar saturation);
        Scalar distanceTo(const ColorDescriptor& rhs) const;

      protected:

        static const std::vector<std::vector<double> > DISTANCE_MATRIX;
        HistoType saturationBalance_;

      private:

        friend std::ostream& operator<<(std::ostream& stream, const ColorDescriptor& rhs);

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & (ColorHistogram) *this;
            ar & saturationBalance_;
        }

    };

    std::ostream& operator<<(std::ostream& stream, const ColorDescriptor& rhs);

  } // namespace
} // namespace

#endif /* SURE_COLOR_DESCRIPTOR_H_ */
