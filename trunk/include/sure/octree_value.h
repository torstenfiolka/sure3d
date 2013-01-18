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

#ifndef OCTREE_VALUE_H_
#define OCTREE_VALUE_H_

#include <sure/normal_histogram.h>

namespace sure {

  //! stores extra data in every octree node
  class OctreeValue
  {
  public:

    enum NormalStatus
    {
      NORMAL_NOT_CALCULATED = 0,
      NORMAL_STABLE,
      NORMAL_UNSTABLE
    };

    enum NodeStatus
    {
      MAXIMUM_NOT_CALCULATED = 0,
      MAXIMUM_POSSIBLE,
      MAXIMUM_FOUND,
      MAXIMUM_NOT_POSSIBLE,
      ARTIFICIAL,
      BACKGROUND,
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OctreeValue(int value = 0) : normalHistogram()
    {
      clear();
    }

    OctreeValue(const OctreeValue& rhs) : normalHistogram()
    {
      if( this != &rhs )
      {
    	this->normalHistogram = sure::NormalHistogram(rhs.normalHistogram);
        this->colorR = rhs.colorR;
        this->colorG = rhs.colorG;
        this->colorB = rhs.colorB;
        this->cornerness3D = rhs.cornerness3D;
        this->entropy = rhs.entropy;
        this->density = rhs.density;
        this->scale = rhs.scale;
        this->normalHistogram = rhs.normalHistogram;
        this->numberOfPoints = rhs.numberOfPoints;
        this->pointCloudIndex = rhs.pointCloudIndex;
        this->statusOfMaximum = rhs.statusOfMaximum;
        this->statusOfNormal = rhs.statusOfNormal;
        for(int i=0; i<3; ++i)
        {
          this->normal[i] = rhs.normal[i];
          this->summedPos[i] = rhs.summedPos[i];
          this->summedSquares[i] = rhs.summedSquares[i];
        }
        for(int i=3; i<9; ++i)
        {
          this->summedSquares[i] = rhs.summedSquares[i];
        }

        for(int i=0; i<5; ++i)
        {
          test[i] = rhs.test[i];
        }
      }
    }

    ~OctreeValue()
    {
    }

    void clear();

    const sure::OctreeValue& operator=(const sure::OctreeValue& rhs);
    sure::OctreeValue operator+(const sure::OctreeValue& rhs) const;
    sure::OctreeValue& operator+=(const sure::OctreeValue& rhs);

    void print() const;

    float r() const;
    float g() const;
    float b() const;

    void calculateHistogram();
    const sure::NormalHistogram& getHistogram() const { return normalHistogram; }
    sure::NormalHistogram& getHistogram() { return normalHistogram; }

    int pointCloudIndex;

    float summedSquares[9];
    float summedPos[3];
    unsigned int numberOfPoints;
    float colorR, colorG, colorB;
    float cornerness3D;

    float entropy;
    float density;
    float scale;

    float normal[3];
    float test[5];

    sure::OctreeValue::NormalStatus statusOfNormal;

    sure::OctreeValue::NodeStatus statusOfMaximum;

  protected:

    sure::NormalHistogram normalHistogram;

  };


} // namespace


#endif /* OCTREE_VALUE_H_ */
