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

#ifndef SURE_3D_H_
#define SURE_3D_H_

#include <sure/data/configuration.h>
#include <sure/memory/fixed_size_allocator.h>

#include <sure/normal/normal_estimation.h>
#include <sure/payload/payload_normal.h>
#include <sure/payload/payload_xyzrgb.h>

#include <sure/data/range_image.h>
#include <sure/octree/octree_node.h>
#include <sure/octree/octree.h>

#include <sure/keypoints/keypoint_calculation.h>
#include <sure/feature/feature_extraction.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace sure
{

  class SUREFeatureExtractor : public pcl::PCLBase<pcl::PointXYZRGB>
  {
    public:

      SUREFeatureExtractor() : verbose(false) { }

      typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

      typedef sure::payload::PointsRGB FixedPayload;
      typedef sure::octree::Node<FixedPayload> Node;
      typedef sure::memory::FixedSizeAllocator<Node> Allocator;

      typedef sure::range_image::RangeImage<pcl::PointXYZRGB> RangeImage;
      typedef sure::octree::Octree<FixedPayload> Octree;

      typedef sure::feature::Feature Feature;

      /**
       * If true, some output will be generated while calculating the features
       */
      bool verbose;

      RangeImage rangeImage;
      Octree octree;
      PointCloud addedPoints;

      /**
       * A vector containing the calculated features
       */
      std::vector<Feature> features;

      /**
       * The configuration used for feature calculation
       */
      Configuration config;

      /**
       * Main function for feature calculation
       * @return true, if features were calculated, false otherwise
       */
      bool calculateSURE();

      /**
       * Returns a pcl pointcloud with the calculated interest points
       * The strength value is used for storing the feature size
       */
      pcl::PointCloud<pcl::InterestPoint>::Ptr getInterestPoints() const;

    protected:

      std::vector<Node*> keypointNodes_;

      bool buildOctree();
      bool calculateNormals();
      bool extractKeypoints();
      bool extractFeatures();

      sure::memory::FixedSizeAllocatorWithDirectAccess<sure::payload::NormalPayload> normalAllocator_;
      sure::memory::FixedSizeAllocatorWithDirectAccess<sure::payload::EntropyPayload> entropyAllocator_;

  };

}

#endif /* SURE_3D_H_ */
