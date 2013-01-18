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

#include <sure/octree_value.h>

void sure::OctreeValue::clear()
{
  for(int i=0; i<3; ++i)
  {
    summedSquares[i] = summedPos[i] = 0.f;
    normal[i] = 0.f;
  }
  for(int i=3; i<9; ++i)
  {
    summedSquares[i] = 0.f;
  }
  numberOfPoints = 0;
  colorR = colorG = colorB = 0.f;
  entropy = cornerness3D =  0.f;

  pointCloudIndex = -1;

  statusOfNormal = sure::OctreeValue::NORMAL_NOT_CALCULATED;

  normalHistogram = NULL;

  statusOfMaximum = sure::OctreeValue::MAXIMUM_NOT_CALCULATED;
}

const sure::OctreeValue& sure::OctreeValue::operator=(const sure::OctreeValue& rhs)
{
  if( this != &rhs )
  {
    this->colorR = rhs.colorR;
    this->colorG = rhs.colorG;
    this->colorB = rhs.colorB;
    this->cornerness3D = rhs.cornerness3D;
    this->entropy = rhs.entropy;
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
    this->entropyHistogram = this->normalHistogram = NULL;

  }
  return *this;
}

sure::OctreeValue sure::OctreeValue::operator+(const sure::OctreeValue& rhs) const
{
  sure::OctreeValue r = *this;
  for(int i=0; i<3; ++i)
  {
    r.summedPos[i] += rhs.summedPos[i];
    r.summedSquares[i] += rhs.summedSquares[i];
  }
  for(int i=3; i<9; ++i)
  {
    r.summedSquares[i] += rhs.summedSquares[i];
  }
  r.colorR += rhs.colorR;
  r.colorG += rhs.colorG;
  r.colorB += rhs.colorB;
  r.numberOfPoints += rhs.numberOfPoints;
  return r;
}

sure::OctreeValue& sure::OctreeValue::operator+=(const sure::OctreeValue& rhs)
{
  for(int i=0; i<3; ++i)
  {
    summedPos[i] += rhs.summedPos[i];
    summedSquares[i] += rhs.summedSquares[i];
  }
  for(int i=3; i<9; ++i)
  {
    summedSquares[i] += rhs.summedSquares[i];
  }
  colorR += rhs.colorR;
  colorG += rhs.colorG;
  colorB += rhs.colorB;
  numberOfPoints += rhs.numberOfPoints;
  return *this;
}

void sure::OctreeValue::print() const
{
  std::cout << "SummedPos: " << summedPos[0] << "/" << summedPos[1] << "/" << summedPos[2] << std::endl;
  std::cout << "SummedSquares: ";
  std::cout << "Punkte: " << numberOfPoints << " statusOfMaximum: " << statusOfMaximum << std::endl;
  std::cout << "Farbe RGB: " << r() << "/" << g() << "/" << b() << std::endl;
  switch( statusOfNormal )
  {
    case sure::OctreeValue::NORMAL_NOT_CALCULATED:
      std::cout << "Keine Normale" << std::endl;
      break;
    case sure::OctreeValue::NORMAL_UNSTABLE:
      std::cout << "Instabile Normale" << std::endl;
      break;
    case sure::OctreeValue::NORMAL_STABLE:
      std::cout << "Normale: " << normal[0] << "/" << normal[1] << "/" << normal[2] << std::endl;
      break;
  }
  switch( statusOfMaximum )
  {
    case sure::OctreeValue::MAXIMUM_NOT_CALCULATED:
      std::cout << "Maximum not calculated." << std::endl;
      break;
    case sure::OctreeValue::MAXIMUM_POSSIBLE:
      std::cout << "Maximum possible." << std::endl;
      break;
    case sure::OctreeValue::MAXIMUM_NOT_POSSIBLE:
      std::cout << "Maximum not possible." << std::endl;
      break;
    case sure::OctreeValue::MAXIMUM_FOUND:
      std::cout << "Maximum found." << std::endl;
      break;
    default:
    case sure::OctreeValue::ARTIFICIAL:
      std::cout << "Artificial Node." << std::endl;
      break;
  }
  if( normalHistogram )
  {
    normalHistogram->print();
  }
}

float sure::OctreeValue::r() const
{
  if( numberOfPoints > 0)
    return colorR / float(numberOfPoints);
  return 0.f;
}

float sure::OctreeValue::g() const
{
  if( numberOfPoints > 0)
    return colorG / float(numberOfPoints);
  return 0.f;
}

float sure::OctreeValue::b() const
{
  if( numberOfPoints > 0)
    return colorB / float(numberOfPoints);
  return 0.f;
}
