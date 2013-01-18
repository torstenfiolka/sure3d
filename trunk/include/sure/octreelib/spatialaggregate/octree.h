/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 4/2011
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute 
 *     VI nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific 
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __SPATIALAGGREGATE_OCTREE_H__
#define __SPATIALAGGREGATE_OCTREE_H__

#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigen>

//#include <sure/octree_value.h>

namespace spatialaggregate {

  template< typename CoordType, typename ValueType > class OcTreeNodeAllocator;

  enum OcTreeNodeType {
    OCTREE_LEAF_NODE,
    OCTREE_BRANCHING_NODE,
    NUM_OCTREE_NODE_TYPES,
  };

  //! position in the octree with templated coordinate type and some operator overloads
  template< typename CoordType >
  class OcTreePosition {
    public:
      OcTreePosition() {}
      OcTreePosition(CoordType x, CoordType y, CoordType z) {
        p[0] = x;
        p[1] = y;
        p[2] = z;
      }
      ~OcTreePosition() {}

      CoordType& operator[](const int rhs) {
        return p[rhs];
      }

      const CoordType& operator[](const int rhs) const {
        return p[rhs];
      }

      OcTreePosition& operator*=(const double &rhs) {
        p[0] *= rhs;
        p[1] *= rhs;
        p[2] *= rhs;
        return *this;
      }

      OcTreePosition& operator*=(const float &rhs) {
        p[0] *= rhs;
        p[1] *= rhs;
        p[2] *= rhs;
        return *this;
      }

      OcTreePosition& operator*=(const int &rhs) {
        p[0] *= rhs;
        p[1] *= rhs;
        p[2] *= rhs;
        return *this;
      }

      const OcTreePosition operator*(const double &rhs) const {
        OcTreePosition r = *this;
        r.p[0] *= rhs;
        r.p[1] *= rhs;
        r.p[2] *= rhs;
        return r;
      }

      const OcTreePosition operator*(const float &rhs) const {
        OcTreePosition r = *this;
        r.p[0] *= rhs;
        r.p[1] *= rhs;
        r.p[2] *= rhs;
        return r;
      }

      const OcTreePosition operator*(const int &rhs) const {
        OcTreePosition r = *this;
        r.p[0] *= rhs;
        r.p[1] *= rhs;
        r.p[2] *= rhs;
        return r;
      }


      OcTreePosition& operator+=(const OcTreePosition &rhs) {
        p[0] += rhs.p[0];
        p[1] += rhs.p[1];
        p[2] += rhs.p[2];
        return *this;
      }

      OcTreePosition& operator-=(const OcTreePosition &rhs) {
        p[0] -= rhs.p[0];
        p[1] -= rhs.p[1];
        p[2] -= rhs.p[2];
        return *this;
      }

      const OcTreePosition operator+(const OcTreePosition &rhs) const {
        OcTreePosition r = *this;
        r += rhs;
        return r;
      }

      const OcTreePosition operator-(const OcTreePosition &rhs) const {
        OcTreePosition r = *this;
        r -= rhs;
        return r;
      }

      bool operator==(const OcTreePosition &rhs) {
        if( p[0] == rhs.p[0] && p[1] == rhs.p[1] && p[2] == rhs.p[2] )
          return true;
        else
          return false;
      }

      CoordType p[3];

  };


  //! point in the octree with templated position and value
  template< typename CoordType, typename ValueType >
  class OcTreePoint {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      OcTreePoint() {}
      OcTreePoint( const OcTreePosition< CoordType >& pos, const ValueType& v ) { position = pos; value = v; }
      ~OcTreePoint() {}

      OcTreePosition< CoordType > position;
      ValueType value;
  };


  /** \brief a node in the octree, either leaf or branching node
   *
   */
  template< typename CoordType, typename ValueType >
  class OcTreeNode {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      OcTreeNode() {
        this->type = NUM_OCTREE_NODE_TYPES;
        this->allocator = NULL;
        initialize();
      }

      OcTreeNode( OcTreeNodeType type ) {
        this->type = type;
        this->allocator = NULL;
        initialize();
      }

      OcTreeNode( OcTreeNodeType type, OcTreeNodeAllocator< CoordType, ValueType >* allocator ) {
        this->type = type;
        this->allocator = allocator;
        initialize();
      }

      void initialize() {
        for( unsigned int i = 0; i < 8; i++ )
          siblings[i] = NULL;
        depth = 0;
        numPoints = 0;
        value = (ValueType) 0;
      }

      ~OcTreeNode() {
        for( unsigned int i = 0; i < 8; i++ ) {
          if( siblings[i] ) {
            allocator->deallocate( siblings[i] );
            siblings[i] = NULL;
          }
        }
      }


      OcTreeNodeType type;

      OcTreeNode* parent;
      OcTreeNode< CoordType, ValueType >* siblings[8];

      OcTreeNodeAllocator< CoordType, ValueType >* allocator;

      // dimensions of the space covered by this node
      OcTreePosition< CoordType > position; // center position
      OcTreePosition< CoordType > closestPosition; // closest point to center in a branching node, else just the position for a leaf
      CoordType closestPositionDistance;
      OcTreePosition< CoordType > minPosition;
      OcTreePosition< CoordType > maxPosition;
      unsigned int depth;
      ValueType value;
      unsigned int numPoints;


      //! position in my region?
      bool inRegion( const OcTreePosition< CoordType >& position );

      //! my center in given region?
      bool inRegion( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition );

      bool overlap( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition );

      bool containedInRegion( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition );

      bool regionContained( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition );

      unsigned int getOctant( const OcTreePosition< CoordType >& position );

      void setDimensionsForParentOctant( unsigned int octant );

      OcTreeNode< CoordType, ValueType >* addPoint( OcTreeNode< CoordType, ValueType >* leaf, CoordType minimumVolumeSize );
      OcTreeNode< CoordType, ValueType >* addPoint( const OcTreePoint< CoordType, ValueType >& point, CoordType minimumVolumeSize );

      unsigned int getPointsInVolume( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      void getAllNodesInVolume( std::vector< OcTreeNode< CoordType, ValueType >* >& points, const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      void getAllNodesInVolumeOnSamplingDepth( std::vector< OcTreeNode< CoordType, ValueType >* >& points, const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, int searchDepth, bool higherDepthLeaves);

      ValueType getValueInVolume( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize );

      void getValueAndCountInVolume( ValueType& value, unsigned int& count, const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize );

      void applyOperatorInVolume( ValueType& value, void* data, void (*f)( ValueType& v, OcTreeNode< CoordType, ValueType >* current, void* data ), const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize );

      OcTreeNode< CoordType, ValueType >* getTightestNode( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize );

      OcTreeNode< CoordType, ValueType >* getNodeOnLevel(const spatialaggregate::OcTreePosition< CoordType >& position, unsigned int level);

      void collectNodesInDepthRange( std::vector< OcTreeNode< CoordType, ValueType >* >& nodes, unsigned int minDepth, unsigned int maxDepth );

      // sweeps up the tree and applies the given function on parent node and this node
      void sweepUp( void* data, void (*f)( OcTreeNode< CoordType, ValueType >* current, OcTreeNode< CoordType, ValueType >* next, void* data ) );

      void sweepDown( void* data, void (*f)( OcTreeNode< CoordType, ValueType >* current, OcTreeNode< CoordType, ValueType >* next, void* data ) );

  };


  /** \brief simple allocator uses new operator
   *
   */
  template< typename CoordType, typename ValueType >
  class OcTreeNodeAllocator {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      OcTreeNodeAllocator() {}
      virtual ~OcTreeNodeAllocator() {}

      virtual OcTreeNode< CoordType, ValueType >* allocateLeafNode() { return new OcTreeNode< CoordType, ValueType >( OCTREE_LEAF_NODE, this ); }
      virtual void deallocateLeafNode( OcTreeNode< CoordType, ValueType >* node ) { delete node; }

      virtual OcTreeNode< CoordType, ValueType >* allocateBranchingNode() { return new OcTreeNode< CoordType, ValueType >( OCTREE_BRANCHING_NODE, this ); }
      virtual void deallocateBranchingNode( OcTreeNode< CoordType, ValueType >* node ) { delete node; }

      virtual void deallocate( OcTreeNode< CoordType, ValueType >* node ) { delete node; }

      virtual void reset() {}
  };


  /** \brief fixed count allocator, pre-allocates memory for a fixed number of points
   *
   */
  template< typename CoordType, typename ValueType >
  class OcTreeNodeFixedCountAllocator : public OcTreeNodeAllocator< CoordType, ValueType > {
    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OcTreeNodeFixedCountAllocator( unsigned int numPoints ) {
      leafNodes = new OcTreeNode< CoordType, ValueType >[ numPoints+1 ];
      for( currentLeafNode = &leafNodes[0]; currentLeafNode != &leafNodes[numPoints+1]; currentLeafNode++ ) {
        currentLeafNode->type = OCTREE_LEAF_NODE;
        currentLeafNode->allocator = this;
      }
      branchingNodes = new OcTreeNode< CoordType, ValueType >[ 2*numPoints+1 ];
      for( currentBranchingNode = &branchingNodes[0]; currentBranchingNode != &branchingNodes[2*numPoints+1]; currentBranchingNode++ ) {
        currentBranchingNode->type = OCTREE_BRANCHING_NODE;
        currentBranchingNode->allocator = this;
      }
      currentLeafNode = &leafNodes[0];
      currentBranchingNode = &branchingNodes[0];
      this->numPoints = numPoints;
    }

    virtual ~OcTreeNodeFixedCountAllocator() {
      delete[] leafNodes;
      delete[] branchingNodes;
    }

    virtual OcTreeNode< CoordType, ValueType >* allocateLeafNode() {
      return currentLeafNode++;
    }
    virtual void deallocateLeafNode( OcTreeNode< CoordType, ValueType >* node ) {}

    virtual OcTreeNode< CoordType, ValueType >* allocateBranchingNode() {
      return currentBranchingNode++;
    }
    virtual void deallocateBranchingNode( OcTreeNode< CoordType, ValueType >* node ) {}

    virtual void deallocate( OcTreeNode< CoordType, ValueType >* node ) {}

    virtual void reset() {
      OcTreeNode< CoordType, ValueType >* lastLeafNode = currentLeafNode;
      for( currentLeafNode = &leafNodes[0]; currentLeafNode != lastLeafNode; currentLeafNode++ )
        currentLeafNode->initialize();

      OcTreeNode< CoordType, ValueType >* lastBranchingNode = currentBranchingNode;
      for( currentBranchingNode = &branchingNodes[0]; currentBranchingNode != lastBranchingNode; currentBranchingNode++ )
        currentBranchingNode->initialize();

      currentLeafNode = &leafNodes[0];
      currentBranchingNode = &branchingNodes[0];
    }

    OcTreeNode< CoordType, ValueType >* leafNodes;
    OcTreeNode< CoordType, ValueType >* branchingNodes;
    OcTreeNode< CoordType, ValueType >* currentLeafNode;
    OcTreeNode< CoordType, ValueType >* currentBranchingNode;
    unsigned int numPoints;
  };


  /** \brief the octree class with some convenient constructors
   *
   */
  template< typename CoordType, typename ValueType >
  class OcTree {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      OcTree( const OcTreePosition< CoordType >& dimensions, const OcTreePosition< CoordType >& center, CoordType minimumVolumeSize = 0, OcTreeNodeAllocator< CoordType, ValueType >* allocator = NULL );

      //! creates a tree in which the node sizes are multiples of minimumVolumeSize with sufficient depth levels to capture points in maxDistance radius around center
      OcTree( const OcTreePosition< CoordType >& center, CoordType minimumVolumeSize, CoordType maxDistance, OcTreeNodeAllocator< CoordType, ValueType >* allocator = NULL );

      ~OcTree();

      void initialize( const OcTreePosition< CoordType >& dimensions, const OcTreePosition< CoordType >& center, CoordType minimumVolumeSize );

      OcTreeNode< CoordType, ValueType >* addPoint( const OcTreePoint< CoordType, ValueType >& point );

      OcTreeNode< CoordType, ValueType >* getClosestPoint( const OcTreePosition< CoordType >& position );

      // complete tree
      unsigned int getPointsInVolume( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      std::vector< OcTreeNode< CoordType, ValueType >* > getAllNodesInVolume( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      // complete tree
      ValueType getValueInVolume( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      // complete tree
      void getValueAndCountInVolume( ValueType& value, unsigned int& count, const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      void applyOperatorInVolume( ValueType& value, void* data, void (*f)( ValueType& v, OcTreeNode< CoordType, ValueType >* current, void* data ), const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize = 0 );

      OcTreeNode< CoordType, ValueType >* getNodeOnLevel(const spatialaggregate::OcTreePosition< CoordType >& position, unsigned int level);

      OcTreeNode< CoordType, ValueType >* root;

      CoordType minimumVolumeSize;

      OcTreeNodeAllocator< CoordType, ValueType >* allocator;

      bool deleteAllocator;

  };



};

#include <sure/octreelib/spatialaggregate/octree.hpp>

#endif //__SPATIALAGGREGATE_OCTREE_H__


