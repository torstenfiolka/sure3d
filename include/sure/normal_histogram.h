// Software License Agreement (BSD License)
//
// Copyright (c) 2012, Fraunhofer FKIE/US
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

#ifndef NORMAL_HISTOGRAM_H_
#define NORMAL_HISTOGRAM_H_

#include <map>
#include <cmath>
#include <Eigen/Dense>
#include <pcl/point_types.h>

// TODO: rethink allocator
// TODO: replace std::map

namespace sure {

//! stores the histogram for a normal
class NormalHistogram {
public:

  enum WeightMethod
  {
    NO_WEIGHT = 0,
    INVERSE_ABSOLUTE_DOT_PRODUCT,
    INVERSE_POSITIVE_DOT_PRODUCT,
    EXCLUSION
  };

  //! constants for the class
  // 4:22 5:34 6:49 7:66 10:134
  static const int NUMBER_OF_BINS = 5;
  static const int HISTOGRAM_SIZE = 34;
  static const float LOG_BASE_2;
  static const float MAX_ENTROPY;
  static const float MAX_DISTANCE;
  static const float EPSILON = 1e-4;

  NormalHistogram(int value = 0);
  NormalHistogram(const sure::NormalHistogram& obj)
  {
    this->operator=(obj);
  }
  ~NormalHistogram();

  //! sets the + operator for adding histograms
  sure::NormalHistogram& operator=(const sure::NormalHistogram& rhs);
  sure::NormalHistogram operator+(const sure::NormalHistogram& rhs) const;
  sure::NormalHistogram& operator+=(const sure::NormalHistogram& rhs);

  //! sets the * operator for normalizing/scaling the histogram
  sure::NormalHistogram operator*(const float& rhs) const;

  //! clears all data
  void clear();

  //! calculates the histogramm for a given normal
  void calculateHistogram(const Eigen::Vector3f& normal, int points = 1);
  void calculateHistogram(const float normal[3], int points = 1);

  void insertCrossProduct(const Eigen::Vector3f& referenceNormal, const Eigen::Vector3f& secondNormal, WeightMethod weightMethod = sure::NormalHistogram::NO_WEIGHT);

  //! calculates the entropy of the histogram
  void calculateEntropy();

  //! returns a pcl::histogram
  pcl::Histogram<sure::NormalHistogram::HISTOGRAM_SIZE> getHistogram() const;

  //! prints the histogram to stdout
  void print() const;

  //! the histogram
  float values[HISTOGRAM_SIZE];

  float entropy;

  unsigned int numberOfPoints;
  float weight;
  float zeroClass;

};

//! Initializes the histogram classes
static std::map<int, Eigen::Vector3f, std::less<int>, Eigen::aligned_allocator<Eigen::Vector3f> > initVectors()
{
  std::map<int, Eigen::Vector3f, std::less<int>, Eigen::aligned_allocator<Eigen::Vector3f> > vecs;
  const float polarBinSize = M_PI / float(sure::NormalHistogram::NUMBER_OF_BINS);
  float azimuthBinSize;
  float polarCenter, azimuthCenter;
  int numberOfAzimuthBins;
  int count = 0;
  for(int polarBin=0; polarBin<sure::NormalHistogram::NUMBER_OF_BINS; ++polarBin)
  {
    polarCenter = polarBinSize * float(polarBin);
    numberOfAzimuthBins = floor(fabs(float(sure::NormalHistogram::NUMBER_OF_BINS*2) * sin(polarCenter))) + 1;
    azimuthBinSize = (2.f * M_PI) / float(numberOfAzimuthBins);
    for(int azimuthBin=0; azimuthBin<numberOfAzimuthBins; ++azimuthBin)
    {
      azimuthCenter = azimuthBinSize * float(azimuthBin);
      vecs[count][0] = sin(polarCenter) * cos(azimuthCenter);
      vecs[count][1] = sin(polarCenter) * sin(azimuthCenter);
      vecs[count][2] = cos(polarCenter);
      count++;
    }
  }
  vecs[count][0] = 0.f;
  vecs[count][1] = 0.f;
  vecs[count][2] = -1.f;
//  ROS_DEBUG("Created normal histogram with %i classes.", (int) vecs.size());
  return vecs;
}

static const std::map<int, Eigen::Vector3f, std::less<int>, Eigen::aligned_allocator<Eigen::Vector3f> > REFERENCE_VECTORS = sure::initVectors();
//const std::map<int, Eigen::Vector3f, std::less<int>, Eigen::aligned_allocator<Eigen::Vector3f> > sure::referenceVectors

}

#endif /* NORMAL_HISTOGRAM_H_ */
