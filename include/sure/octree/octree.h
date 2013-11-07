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

#ifndef SURE_OCTREE_H_
#define SURE_OCTREE_H_

#include <vector>
#include <map>
#include <deque>
#include <iomanip>
#include <cmath>
#include <climits>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sure/data/typedef.h>
#include <sure/access/region.h>
#include <sure/data/range_image.h>
#include <sure/octree/octree_node.h>
#include <sure/memory/fixed_size_allocator.h>

#include <sure/payload/payload_xyzrgb.h>

namespace sure
{
  namespace octree
  {

    typedef sure::access::Point Point;
    typedef sure::access::Region Region;
    typedef sure::OctantType OctantType;

    template <typename FixedPayloadT>
    class Octree;

    template <typename FixedPayloadT>
    std::ostream& operator<<(std::ostream& stream, const Octree<FixedPayloadT>& rhs);

    /**
     * Octree data structure
     * Its internal addressing is integerbased
     * During the building, each node will be added to a list corresponding to its depth
     */
    template <typename FixedPayloadT>
    class Octree
    {
      public:

        typedef sure::octree::Node<FixedPayloadT> Node;
        typedef sure::memory::FixedSizeAllocator<Node> Allocator;
        typedef std::vector<Node* > NodeVector;
        typedef std::map<unsigned, NodeVector> LevelMap;

        Octree() : root_(NULL), allocator_(), maxDepth_(0), minimumNodeSize_(DEFAULT_MINIMUM_NODE_SIZE), maxNodeResolution_(DEFAULT_MINIMUM_NODE_SIZE/(Scalar) DEFAULT_MIN_NODE_UNIT_SIZE), octreeCenter_(Vector3::Zero()), initialized_(false)
        {

        }

        ~Octree() { }

        void clear();

        bool initialize(Scalar minNodeSize, Scalar expansion, unsigned capacity, const Vector3& center);

        /**
         * Adds a pointcloud to the octree
         * @param cloud
         */
        template <typename PointT>
        void addPointCloud(const pcl::PointCloud<PointT>& cloud);

        /**
         * Adds a pointcloud to the octree incorporating depth border information
         * @param cloud
         * @param rangeImage Provides depth border information
         */
        template <typename PointT>
        void addPointCloud(const pcl::PointCloud<PointT>& cloud, const sure::range_image::RangeImage<PointT>& rangeImage);

        /**
         * Adds a pointcloud to the octree flagged as artificial points
         * @param cloud
         */
        template <typename PointT>
        void addArtificialPointCloud(const pcl::PointCloud<PointT>& cloud);

        //! Maximum octree depth
        unsigned getMaximumDepth() const { return maxDepth_; }

        //! Edge length of the smallest leaf
        float getMinimumNodeSize() const { return minimumNodeSize_; }

        //! Center of the octree
        const Vector3& getCenter() const { return octreeCenter_; }

        /**
         * Return the list of nodes at a given depth. Throws out_of_range, if depth exceeds max depth
         * @param depth
         * @return
         */
        const NodeVector& at(unsigned depth) const
        {
          if( depth > maxDepth_ )
          {
            throw(std::out_of_range("Requested depth is greater than octree depth."));
          }
          return map_.at(depth);
        }
        NodeVector& at(unsigned depth)
        {
          if( depth > maxDepth_ )
          {
            throw(std::out_of_range("Requested depth is greater than octree depth."));
          }
          return map_.at(depth);
        }

        /**
         * Returns the list of nodes at a given depth. Does not check bounds
         * @param depth
         * @return
         */
        const NodeVector& operator[](unsigned depth) const { return map_[depth]; }
        NodeVector& operator[](unsigned depth) { return map_[depth]; }

        /**
         * Returns a vector with all nodes which are fully included in a specified area.
         */
        NodeVector getNodes(const Point& a, unsigned radius) const
        {
          return getNodes(Region(a-radius, a+radius));
        }
        NodeVector getNodes(const Vector3& point, Scalar radius) const
        {
          return getNodes(Region(getAddress(point), getUnitSize(radius)));
        }
        NodeVector getNodes(const Region& r) const;

        /**
         * Returns a vector with all nodes in a given depth which overlap with a specified area. Does not work for depth == 0.
         */
        NodeVector getNodes(const Point& a, unsigned radius, unsigned depth) const
        {
          return getNodes(Region(a-radius, a+radius), depth);
        }
        NodeVector getNodes(const Vector3& point, Scalar radius, Scalar samplingrate) const
        {
          return getNodes(Region(getAddress(point), getUnitSize(radius)), getDepth(samplingrate));
        }
        NodeVector getNodes(const Region& r, unsigned depth) const;

        /**
         * Returns a vector with the neighbor nodes to a given node (in the same depth).
         */
        NodeVector getNodes(Node* node, unsigned radius) const { return getNodes(node->center(), radius, node->depth()); }

        /**
         * Integrates the fixed payload in a given area.
         */
        unsigned integratePayload(const Point& a, unsigned radius, FixedPayloadT& payload) const { return integratePayload(Region (a-radius, a+radius), payload); }
        unsigned integratePayload(const Vector3& point, Scalar radius, FixedPayloadT& payload) const
        {
          return integratePayload(Region(getAddress(point), getUnitSize(radius)), payload);
        }
        unsigned integratePayload(const Point& a, unsigned radius, unsigned depth, FixedPayloadT& payload) const { return integratePayload(Region (a-radius, a+radius), depth, payload); }
        unsigned integratePayload(const Vector3& point, Scalar radius, Scalar samplingrate, FixedPayloadT& payload) const
        {
          return integratePayload(Region(getAddress(point), getUnitSize(radius)), getDepth(samplingrate), payload);
        }
        unsigned integratePayload(const Region& r, FixedPayloadT& payload) const;
        unsigned integratePayload(const Region& r, unsigned depth, FixedPayloadT& payload) const;

        /**
         * Integrates the optional payload in a given area.
         * NOTE: The optional payload type needs to have an operator+= defined.
         * NOTE: static_cast is used for typecasting the pointer
         */
        template <typename OptionalPayloadT>
        unsigned integrateOptionalPayload(const Point& a, unsigned radius, OptionalPayloadT& payload) const { return integrateOptionalPayload<OptionalPayloadT>(Region (a-radius, a+radius), payload); }
        template <typename OptionalPayloadT>
        unsigned integrateOptionalPayload(const Vector3& point, Scalar radius, OptionalPayloadT& payload) const
        {
          return integrateOptionalPayload(Region(getAddress(point), getUnitSize(radius)), payload);
        }
        template <typename OptionalPayloadT>
        unsigned integrateOptionalPayload(const Point& a, unsigned radius, unsigned depth, OptionalPayloadT& payload) const{ return integrateOptionalPayload<OptionalPayloadT>(Region (a-radius, a+radius), depth, payload); }
        template <typename OptionalPayloadT>
        unsigned integrateOptionalPayload(const Vector3& point, Scalar radius, Scalar samplingrate, OptionalPayloadT& payload) const
        {
          return integrateOptionalPayload(Region(getAddress(point), getUnitSize(radius)), getDepth(samplingrate), payload);
        }
        template <typename OptionalPayloadT>
        unsigned integrateOptionalPayload(const Region& r, OptionalPayloadT& payload) const;
        template <typename OptionalPayloadT>
        unsigned integrateOptionalPayload(const Region& r, unsigned depth, OptionalPayloadT& payload) const;

        /**
         * Returns a pointer to the node which contains the given Address on a given depth. If depth ist not set or zero, the maximum depth will be assumed.
         * If no node contains the Address, NULL will be returned.
         */
        Node* getNode(float x, float y, float z, unsigned depth = 0) const { return getNode(getAddress(x, y, z), depth); }
        Node* getNode(const Vector3& p, unsigned depth = 0) const { return getNode(getAddress(p), depth); }
        Node* getNode(const Point& a, unsigned depth = 0) const;

        /**
         * Returns a pointer to the node next the given Address on a given level. If level ist not set or zero, the maximum level will be assumed.
         * If no node is found, the root node will be returned
         * WARNING: May be slow, if the given point is far away from the next node.
         */
        Node* getNextNode(float x, float y, float z, unsigned depth = 0) const { return getNextNode(getAddress(x, y, z), depth); }
        Node* getNextNode(const Vector3& p, unsigned depth = 0) const { return getNextNode(getAddress(p), depth); }
        Node* getNextNode(const Point& a, unsigned depth = 0) const;

        /**
         * Returns the octree address next to a given position
         * @param p
         * @return
         */
        Point getAddress(const Vector3& p) const
        {
          Vector3 pos = (p + octreeCenter_) / maxNodeResolution_;
          return (Point(floor(pos[0]), floor(pos[1]), floor(pos[2])) );
        }
        Point getAddress(float x, float y, float z) const
        {
          return (Point(floor( (x + octreeCenter_[0]) / maxNodeResolution_ ),
                          floor( (y + octreeCenter_[1]) / maxNodeResolution_ ),
                          floor( (z + octreeCenter_[2]) / maxNodeResolution_ )) );
        }

        /**
         * Returns the octree region next to the position in a given depth
         * @param p
         * @param depth Specifies the Depth. If unspecified, max depth will be assumed
         * @return
         */
        Region getRegion(const Vector3& p, unsigned depth = MAX_DEPTH_PLACEHOLDER) const { return getRegion(getAddress(p), depth); }
        Region getRegion(float x, float y, float z, unsigned depth = MAX_DEPTH_PLACEHOLDER) const { return getRegion(getAddress(x,y,z), depth); }
        Region getRegion(const Point& a, unsigned depth = MAX_DEPTH_PLACEHOLDER) const
        {
          Region ret(root_->region());
          unsigned targetDimSize = (depth == MAX_DEPTH_PLACEHOLDER) ? DEFAULT_MIN_NODE_UNIT_SIZE : getUnitSizeFromDepth(depth);
          while( ret.size() != targetDimSize )
          {
            ret = ret.getOctant(ret.getOctant(a));
          }
          return ret;
        }

        /**
         * Returns the size of an octree region in units in a given depth
         * @param depth
         */
        unsigned getUnitSizeFromDepth(unsigned depth) const { return (DEFAULT_MIN_NODE_UNIT_SIZE << maxDepth_-depth); }

        /**
         * Returns the unit size to a given size in meters
         * @param size
         */
        unsigned getUnitSize(Scalar size) const
        {
          unsigned units = floor((size / minimumNodeSize_) + 0.5f);
          return (units * DEFAULT_MIN_NODE_UNIT_SIZE);
        }

        /**
         * Returns the size in meters of an octree node in a given depth
         * @param depth
         * @return
         */
        Scalar getSizeFromDepth(unsigned depth) const
        {
          return ((Scalar) getUnitSizeFromDepth(depth) * maxNodeResolution_);
        }

        /**
         * Returns the size in meters of a given region in the octree
         * @param r
         * @return
         */
        Scalar getSize(const Region& r) const
        {
          return ((Scalar) r.size()) * maxNodeResolution_;
        }

        /**
         * Returns the depth corresponding to a given edge length of the nodes
         * @param size
         */
        unsigned getDepth(Scalar size) const
        {
          int depth = floor(size / minimumNodeSize_);
          return std::min(maxDepth_-(intlog2(depth)), maxDepth_);
        }

        /**
         * Returns the real coordinates to an octree address p
         * @param p
         * @return
         */
        Vector3 getCoords(const Point& p)
        {
          Vector3 v;
          v[0] = (Scalar(p.x()) * (minimumNodeSize_ / Scalar(DEFAULT_MIN_NODE_UNIT_SIZE)));
          v[1] = (Scalar(p.y()) * (minimumNodeSize_ / Scalar(DEFAULT_MIN_NODE_UNIT_SIZE)));
          v[2] = (Scalar(p.z()) * (minimumNodeSize_ / Scalar(DEFAULT_MIN_NODE_UNIT_SIZE)));
          return (v-octreeCenter_);
        }

      protected:

        void insertNode(Node* current, const Node& node, unsigned level);

        unsigned intlog2(unsigned val) const
        {
          unsigned ret(0);
          while( val >>= 1 )
          {
            ret++;
          }
          return ret;
        }

        Node* root_;
        LevelMap map_;

        Allocator allocator_;

        unsigned maxDepth_;
        static const unsigned MAX_DEPTH_PLACEHOLDER = UINT_MAX;

        Scalar minimumNodeSize_, maxNodeResolution_;
        Vector3 octreeCenter_;
        bool initialized_;

      private:

        Octree(const Octree& rhs) { }
        friend std::ostream& operator<< <>(std::ostream& stream, const Octree& rhs);

    };
  }
}

#include <sure/octree/impl/octree.hpp>

#endif /* SURE_OCTREE_H_ */
