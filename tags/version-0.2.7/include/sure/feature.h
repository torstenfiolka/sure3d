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

#ifndef FEATURE_H_
#define FEATURE_H_

#include <cmath>

#include "sure/color_surflet.h"
#include "sure/point_feature_descriptor.h"
#include "sure/lightness_descriptor.h"
#include "sure/hue_descriptor.h"
#include "sure/octree_value.h"

#include "sure/octreelib/spatialaggregate/octree.h"

namespace sure
{

typedef spatialaggregate::OcTreeNode<float, sure::OctreeValue> OctreeNode;

class Feature : public sure::ColorSurflet
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Feature(unsigned int descriptorCount = 1)
  : mEntropy(0.f), mRadius(0.f), mPointCloudIndex(-1), mCornerness3D(0.f),
    numberOfDescriptors(descriptorCount), pfDescriptor(descriptorCount), colorDescriptor(descriptorCount), lightnessDescriptor(descriptorCount) { }

  Feature(const Eigen::Vector3f& pos, unsigned int descriptorCount = 1)
  : sure::ColorSurflet(pos), mEntropy(0.f), mRadius(0.f), mPointCloudIndex(-1), mCornerness3D(0.f),
    numberOfDescriptors(descriptorCount), pfDescriptor(descriptorCount), colorDescriptor(descriptorCount), lightnessDescriptor(descriptorCount) { }

  Feature(const sure::Surflet& rhs, unsigned int descriptorCount = 1)
  : sure::ColorSurflet(rhs), mEntropy(0.f), mRadius(0.f), mPointCloudIndex(-1), mCornerness3D(0.f),
    numberOfDescriptors(descriptorCount), pfDescriptor(descriptorCount), colorDescriptor(descriptorCount), lightnessDescriptor(descriptorCount) { }

  Feature(const sure::ColorSurflet& rhs, unsigned int descriptorCount = 1)
  : sure::ColorSurflet(rhs), mEntropy(0.f), mRadius(0.f), mPointCloudIndex(-1), mCornerness3D(0.f),
    numberOfDescriptors(descriptorCount), pfDescriptor(descriptorCount), colorDescriptor(descriptorCount), lightnessDescriptor(descriptorCount) { }

  Feature(const Eigen::Vector3f& pos, const Eigen::Vector3f& normal, unsigned int descriptorCount = 1)
  : sure::ColorSurflet(pos, normal), mEntropy(0.f), mRadius(0.f), mPointCloudIndex(-1), mCornerness3D(0.f),
    numberOfDescriptors(descriptorCount), pfDescriptor(descriptorCount), colorDescriptor(descriptorCount), lightnessDescriptor(descriptorCount) { }

  sure::Feature operator+(const sure::Feature& rhs) const;
  sure::Feature& operator+=(const sure::Feature& rhs);
  sure::Feature operator *(const double rhs) const;
  sure::Feature& operator *=(const double rhs);
  sure::Feature operator /(const double rhs) const;
  sure::Feature& operator /=(const double rhs);

  ~Feature() {}

  virtual void reset();
  virtual void print() const;

  bool hasNormalizedDescriptors() const;

  double distanceTo(const sure::Feature& rhs, double shapeWeight, double colorWeight, double lightnessWeight) const;
  double distanceTo(const sure::Feature& rhs) const;

  int determineDistanceClass(const sure::Surflet& surflet);
  void calculateDescriptor(const std::vector<sure::ColorSurflet>& surflets, bool normalize = true);
  void calculateDescriptor(const std::vector<OctreeNode* >& nodes, bool normalize = true);

  void createRandomDescriptor();

  void setRadius(float r) { mRadius = r; }
  void setEntropy(float e) { mEntropy = e; }
  void setPointCloudIndex(int i) { mPointCloudIndex = i; }
  void setCornerness(float c) { mCornerness3D = c; }

  float entropy() const { return mEntropy; }
  float radius() const { return mRadius; }
  int getPointCloudIndex() const { return mPointCloudIndex; }
  float getCornerness() const { return mCornerness3D; }

protected:

  float mEntropy;
  float mRadius;
  int mPointCloudIndex;
  float mCornerness3D;

  unsigned int numberOfDescriptors;

  std::vector<sure::PointFeatureDescriptor> pfDescriptor;
  std::vector<sure::HueDescriptor> colorDescriptor;
  std::vector<sure::RelativeLightnessDescriptor> lightnessDescriptor;

//  friend std::ofstream& operator<<(std::ofstream& stream, const sure::Feature& rhs);

private:

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<sure::ColorSurflet>(*this);
    ar & mEntropy;
    ar & mRadius;
    ar & mPointCloudIndex;
    ar & mCornerness3D;
    ar & numberOfDescriptors;
    for(unsigned int i=0; i<numberOfDescriptors; ++i)
    {
      ar & pfDescriptor.at(i);
      ar & colorDescriptor.at(i);
      ar & lightnessDescriptor.at(i);
    }
  }

};

//std::ofstream& operator<<(std::ofstream& stream, const sure::Feature& rhs);

}

#endif /* FEATURE_H_ */
