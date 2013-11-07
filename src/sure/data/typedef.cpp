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

#include <sure/data/typedef.h>
#include <iostream>

const int sure::getNormalHistogramSize(unsigned numberOfPolarBins)
{
  const Scalar polarBinSize = M_PI / (Scalar) numberOfPolarBins;
  int histogramSize = 0;

  for(int polarBin=0; polarBin<numberOfPolarBins; ++polarBin)
  {
    Scalar polarCenter = polarBinSize * (Scalar) polarBin;
    int numberOfAzimuthBins = floor(fabs((Scalar) (numberOfPolarBins*2) * sin(polarCenter))) + 1;
    histogramSize += numberOfAzimuthBins;
  }
  return histogramSize;
}

const sure::Scalar sure::getNormalHistogramBinSize(unsigned numberOfPolarBins)
{
  const Scalar polarBinSize = M_PI / (Scalar) numberOfPolarBins;
  Scalar avgBinSize = 0.0;
  int histogramSize = 0;

  std::cout << "Polar bin size: " << polarBinSize << "\n";

  for(int polarBin=1; polarBin<numberOfPolarBins; ++polarBin)
  {
    Scalar polarCenter = polarBinSize * float(polarBin);
    int numberOfAzimuthBins = floor(fabs(float(numberOfPolarBins*2) * sin(polarCenter))) + 1;
    Scalar azimuthBinSize = (2.f * M_PI * sin(polarCenter)) / float(numberOfAzimuthBins);
    histogramSize += numberOfAzimuthBins;
    avgBinSize += azimuthBinSize * (Scalar) numberOfAzimuthBins;
    std::cout << "Polar bin " << polarBin << " azimuth bin size: " << azimuthBinSize << "\n";
  }
  std::cout << "Average azimuth bin size: " << (avgBinSize / (Scalar) histogramSize) << "\n";
  return (avgBinSize / (Scalar) histogramSize);
}

