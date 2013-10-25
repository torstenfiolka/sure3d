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

#ifndef SURE_ESTIMATOR_H_
#define SURE_ESTIMATOR_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>

#include <boost/smart_ptr.hpp>

#include <sure/octreelib/spatialaggregate/octree.h>
#include <sure/octreelib/algorithm/downsample.h>

#include <sure/octree_value.h>
#include <sure/normal_histogram.h>
#include <sure/configuration.h>
#include <sure/color_surflet.h>
#include <sure/range_image.h>
#include <sure/feature.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace sure
{
  typedef spatialaggregate::OcTree<float,sure::OctreeValue> Octree;
  typedef spatialaggregate::OcTreePosition<float> OctreePosition;
  typedef spatialaggregate::OcTreePoint<float, sure::OctreeValue> OctreePoint;
  typedef spatialaggregate::OcTreeNodeFixedCountAllocator<float, sure::OctreeValue> OctreeAllocator;
//  typedef spatialaggregate::OcTreeNode<float, sure::OctreeValue> OctreeNode;

  const unsigned int STANDARD_OCTREE_SIZE = 500000;
  const unsigned int MINIMUM_POINTS_FOR_NORMAL = 5;

  float calculateCornerness(const std::vector<OctreeNode*>& nodes);

  template<typename Derived>
  bool is_finite(const Eigen::MatrixBase<Derived>& x)
  {
     return ( (x - x).array() == (x - x).array()).all();
  }

  template <typename PointT>
  class SURE_Estimator : public pcl::PCLBase<PointT>
  {
    public:

    using pcl::PCLBase<PointT>::input_;
    using pcl::PCLBase<PointT>::indices_;
    using pcl::PCLBase<PointT>::initCompute;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //
    //  Methods for data and organisational purposes
    //

    /**
     * Constructor
     * @param maxNodes sets the maximum number of octree nodes
     */
    SURE_Estimator(unsigned int maxNodes = sure::STANDARD_OCTREE_SIZE) : octree(NULL), octreeAllocator(NULL)
    {
      currentOctreeSize = 0;
      resize(maxNodes);
    }
    ~SURE_Estimator()
    {
      if( octree )
        delete octree;
      if( octreeAllocator )
        delete octreeAllocator;
    }
    void reset();
    void resize(unsigned int size);

    //
    //  Getters and Setters
    //

    void setConfig(const sure::Configuration& newConf) { this->config = newConf; }

    const pcl::PointCloud<PointT>& getAddedPoints() const { return addedPoints; }
    int getOctreeDepth() { return octreeDepth; }
    std::vector<OctreeNode* >& getOctreeLevelRef(int level) { return octreeMap[level]; }
    std::vector<OctreeNode* >& getOctreeLevelRefByResolution(float resolution) { return octreeMap[config.getSamplingMapIndex(resolution)]; }

    OctreeNode* getNodeFromPosition(int x, int y, int level = 0) { return getNodeFromPosition(input_->points[y*input_->width+x], level); }
    OctreeNode* getNodeFromPosition(const PointT& point, int level = 0);
    OctreeNode* getNodeFromPosition(const Eigen::Vector3f& posVec, int level = 0);
    OctreeNode* getNodeFromPosition(const OctreePosition& pos, int level = 0);

    const sure::RangeImage<PointT> getRangeImage() const { return rangeImage; }
    sure::Octree* getOctree() { return octree; }

    const std::vector<sure::Feature>& getFeatures() const { return features; }
    pcl::PointCloud<pcl::InterestPoint>::Ptr getInterestPoints() const;

    const sure::Configuration& getConfig() const { return config; }

    //
    //  Methodes for the Octree
    //

    void insertPointInOctree(const PointT& cloud, sure::OctreeValue::NodeStatus = sure::OctreeValue::MAXIMUM_NOT_CALCULATED);
    void buildOctree();
    inline void resampleOctreeSamplingMap();
    void continueDepthBorders();

    //
    // Methodes for Normal
    //

    void calculateNormals();
    void calculateNormals(unsigned int level, float radius);
    bool calculateNormal(sure::OctreeNode* node, float radius = 0.f);
    bool calculateNormal(const Eigen::Vector3f& position, float radius, Eigen::Vector3f& normal);

    void orientateNormal(Eigen::Vector3f& normal, const Eigen::Vector3f& position);
    void orientateNormal(Eigen::Vector3f& normal, const OctreePosition& position);
    void orientateNormal(float& normal1, float& normal2, float& normal3, const OctreePosition& position);

    //
    //  Feature calculation
    //

    void calculateFeatures();
    void calculateEntropy();
    void calculateEntropy(unsigned int level, float radius);
    void calculateEntropy(unsigned int level, float radius, unsigned int index);
    float calculateEntropy(OctreeNode* treenode, float radius);
    void extractFeature();

    void localizeFeatureWithMeanShift(int iterations = 3);

//    float calculate(const sure::OctreePosition& position, float radius);
//    void test();

    sure::Feature createFeature(OctreeNode* node);
    sure::Feature createFeature(const Eigen::Vector3f& point);
    sure::Feature createFeature(int index);
    sure::Feature createFeature(int x, int y) { return this->createFeature(y*input_->width+x); }

    void createDescriptor(sure::Feature& feature);

    //
    // Constants and other Stuff
    //

    protected:

    //! stores the additional points
    pcl::PointCloud<PointT> addedPoints;

    //! the octree and its allocator
    Octree* octree;
    OctreeAllocator* octreeAllocator;

    //! the octree sampling map
    algorithm::OcTreeSamplingMap<float, sure::OctreeValue> octreeMap;

    //! size and depth of the octree
    unsigned int octreeDepth, octreeSize, currentOctreeSize;

    //! the size of the octree nodes corresponding to their depths
    std::vector<float> octreeNodeSizeByDepth;

    //! the current configuration
    sure::Configuration config;

    std::vector<sure::Feature > features;

    sure::RangeImage<PointT> rangeImage;

  };

}

#include "sure/impl/sure_estimator.hpp"
//#include "sure/impl/current.hpp"

#endif /* SURE_ESTIMATOR_H_ */
