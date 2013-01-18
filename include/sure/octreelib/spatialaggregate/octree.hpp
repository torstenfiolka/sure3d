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

#include <cmath>

#define OCTREE_EPSILON 0.1f // in relation to minresolution


template< typename CoordType, typename ValueType >
inline unsigned int spatialaggregate::OcTreeNode< CoordType, ValueType >::getOctant( const spatialaggregate::OcTreePosition< CoordType >& query ) {

	if( query[0] < position[0] ) {

		if( query[1] < position[1] ) {

			if( query[2] < position[2] ) {

				return 0;

			}
			else { // query[2] >= position[2]

				return 1;

			}

		}
		else { // query[1] >= position[1]

			if( query[2] < position[2] ) {

				return 2;

			}
			else { // query[2] >= position[2]

				return 3;

			}

		}

	}
	else { // query[0] >= position[0]

		if( query[1] < position[1] ) {

			if( query[2] < position[2] ) {

				return 4;

			}
			else { // query[2] >= position[2]

				return 5;

			}

		}
		else { // query[1] >= position[1]

			if( query[2] < position[2] ) {

				return 6;

			}
			else { // query[2] >= position[2]

				return 7;

			}

		}

	}

}


// TODO: use overloaded function for halving, could be done faster by bitshift for integer coord types
template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::setDimensionsForParentOctant( unsigned int octant ) {

	switch( octant ) {

		case 0: // < < <
			minPosition = parent->minPosition;
			maxPosition = (parent->minPosition + parent->maxPosition) * 0.5f;
			break;

		case 1: // < < >
			minPosition[0] = parent->minPosition[0];
			maxPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			minPosition[1] = parent->minPosition[1];
			maxPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			minPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			maxPosition[2] = parent->maxPosition[2];
			break;

		case 2: // < > <
			minPosition[0] = parent->minPosition[0];
			maxPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			minPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			maxPosition[1] = parent->maxPosition[1];
			minPosition[2] = parent->minPosition[2];
			maxPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			break;

		case 3: // < > >
			minPosition[0] = parent->minPosition[0];
			maxPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			minPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			maxPosition[1] = parent->maxPosition[1];
			minPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			maxPosition[2] = parent->maxPosition[2];
			break;

		case 4: // > < <
			minPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			maxPosition[0] = parent->maxPosition[0];
			minPosition[1] = parent->minPosition[1];
			maxPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			minPosition[2] = parent->minPosition[2];
			maxPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			break;

		case 5: // > < >
			minPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			maxPosition[0] = parent->maxPosition[0];
			minPosition[1] = parent->minPosition[1];
			maxPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			minPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			maxPosition[2] = parent->maxPosition[2];
			break;

		case 6: // > > <
			minPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			maxPosition[0] = parent->maxPosition[0];
			minPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			maxPosition[1] = parent->maxPosition[1];
			minPosition[2] = parent->minPosition[2];
			maxPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			break;

		case 7: // > > >
			minPosition[0] = (parent->minPosition[0] + parent->maxPosition[0]) * 0.5f;
			maxPosition[0] = parent->maxPosition[0];
			minPosition[1] = (parent->minPosition[1] + parent->maxPosition[1]) * 0.5f;
			maxPosition[1] = parent->maxPosition[1];
			minPosition[2] = (parent->minPosition[2] + parent->maxPosition[2]) * 0.5f;
			maxPosition[2] = parent->maxPosition[2];
			break;
	};

}


template< typename CoordType, typename ValueType >
inline bool spatialaggregate::OcTreeNode< CoordType, ValueType >::inRegion( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition ) {

	for( unsigned int i = 0; i < 3; i++ )
		if( position[i] < minPosition[i] || position[i] >= maxPosition[i] )
			return false;
	return true;
}

template< typename CoordType, typename ValueType >
inline bool spatialaggregate::OcTreeNode< CoordType, ValueType >::inRegion( const spatialaggregate::OcTreePosition< CoordType >& position ) {

	for( unsigned int i = 0; i < 3; i++ )
		if( position[i] < minPosition[i] || position[i] > maxPosition[i] )
			return false;
	return true;
}

template< typename CoordType, typename ValueType >
inline bool spatialaggregate::OcTreeNode< CoordType, ValueType >::overlap( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition ) {

	for( unsigned int i = 0; i < 3; i++ ) {
		if( maxPosition[i] < this->minPosition[i] )
			return false;
		if( minPosition[i] > this->maxPosition[i] )
			return false;
	}

	return true;

}


template< typename CoordType, typename ValueType >
inline bool spatialaggregate::OcTreeNode< CoordType, ValueType >::containedInRegion( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition ) {

	if( !overlap( minPosition, maxPosition ) )
		return false;

	for( unsigned int i = 0; i < 3; i++ ) {
		if( this->minPosition[i] < minPosition[i] )
			return false;
		if( this->maxPosition[i] > maxPosition[i] )
			return false;
	}

	return true;
}


template< typename CoordType, typename ValueType >
inline bool spatialaggregate::OcTreeNode< CoordType, ValueType >::regionContained( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition ) {

	if( !overlap( minPosition, maxPosition ) )
		return false;

	for( unsigned int i = 0; i < 3; i++ ) {
		if( minPosition[i] < this->minPosition[i] )
			return false;
		if( maxPosition[i] > this->maxPosition[i] )
			return false;
	}

	return true;
}

template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::sweepUp( void* data, void (*f)( spatialaggregate::OcTreeNode< CoordType, ValueType >* current, spatialaggregate::OcTreeNode< CoordType, ValueType >* next, void* data ) ) {

	f( this, parent, data );

	if( parent ) {
		parent->sweepUp( data, f );
	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::getPointsInVolume( std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* >& points, const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	if( type == OCTREE_LEAF_NODE ) {

		// check if point in leaf is within region
		if( inRegion( minPosition, maxPosition ) ) {
			points.push_back( this );
		}

	}
	else {

		if( (this->maxPosition[0] - this->minPosition[0]) - minimumSearchVolumeSize <= -OCTREE_EPSILON*minimumSearchVolumeSize ) {
			return;
		}

		// for all siblings
		// - if regions overlap: call add points
		for( unsigned int i = 0; i < 8; i++ ) {
			if( !siblings[i] )
				continue;

			if( siblings[i]->overlap( minPosition, maxPosition ) )
				siblings[i]->getPointsInVolume( points, minPosition, maxPosition, minimumSearchVolumeSize );
		}

	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::getAllNodesInVolume( std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* >& points, const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	if( type == OCTREE_LEAF_NODE ) {

		// check if point in leaf is within region
		if( inRegion( minPosition, maxPosition ) ) {
			points.push_back( this );
		}

	}
	else {

		points.push_back(this);

		if( (this->maxPosition[0] - this->minPosition[0]) - minimumSearchVolumeSize <= -OCTREE_EPSILON*minimumSearchVolumeSize ) {
			return;
		}

		// for all siblings
		// - if regions overlap: call add points
		for( unsigned int i = 0; i < 8; i++ ) {
			if( !siblings[i] )
				continue;

			if( siblings[i]->overlap( minPosition, maxPosition ) )
				siblings[i]->getAllNodesInVolume( points, minPosition, maxPosition, minimumSearchVolumeSize );
		}

	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::getAllNodesInVolumeOnSamplingDepth( std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* >& points, const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, int searchDepth, bool higherDepthLeaves) {

	if( (int) depth > searchDepth )
		return;

	if( type == OCTREE_LEAF_NODE ) {

		if( !higherDepthLeaves && (int) depth != searchDepth )
			return;

		// check if point in leaf is within region
		if( inRegion( minPosition, maxPosition ) ) {
			points.push_back( this );
		}

	}
	else {

		if( (int) depth == searchDepth ) {
			points.push_back( this );
			return;
		}

		// for all siblings
		// - if regions overlap: call functino for the sibling
		for( unsigned int i = 0; i < 8; i++ ) {
			if( !siblings[i] )
				continue;

			if( siblings[i]->overlap( minPosition, maxPosition ) )
				siblings[i]->getAllNodesInVolumeOnSamplingDepth( points, minPosition, maxPosition, searchDepth, higherDepthLeaves );
		}
	}

}


template< typename CoordType, typename ValueType >
inline ValueType spatialaggregate::OcTreeNode< CoordType, ValueType >::getValueInVolume( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	if( type == OCTREE_LEAF_NODE ) {

		if( inRegion( minPosition, maxPosition ) )
			return value;

		return ValueType(0);

	}
	else {

		if( !overlap( minPosition, maxPosition ) )
			return ValueType(0);

		if( containedInRegion( minPosition, maxPosition ) )
			return value;

		if( (this->maxPosition[0] - this->minPosition[0]) - minimumSearchVolumeSize <= -OCTREE_EPSILON*minimumSearchVolumeSize ) {
			return value;
		}

		ValueType value = ValueType(0);
		for( unsigned int i = 0; i < 8; i++ ) {
			if(!siblings[i])
				continue;
			value += siblings[i]->getValueInVolume( minPosition, maxPosition, minimumSearchVolumeSize );
		}

		return value;

	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::getValueAndCountInVolume( ValueType& value, unsigned int& count, const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	if( type == OCTREE_LEAF_NODE ) {

		if( inRegion( minPosition, maxPosition ) ) {
			value += this->value;
			count += this->numPoints;
		}

	}
	else {

		if( !overlap( minPosition, maxPosition ) )
			return;

		if( containedInRegion( minPosition, maxPosition ) ) {
			value += this->value;
			count += this->numPoints;
			return;
		}

		if( (this->maxPosition[0] - this->minPosition[0]) - minimumSearchVolumeSize <= -OCTREE_EPSILON * minimumSearchVolumeSize ) {
			value += this->value;
			count += this->numPoints;
			return;
		}

		for( unsigned int i = 0; i < 8; i++ ) {
			if(!siblings[i])
				continue;

			siblings[i]->getValueAndCountInVolume( value, count, minPosition, maxPosition, minimumSearchVolumeSize );

		}

	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::applyOperatorInVolume( ValueType& value, void* data, void (*f)( ValueType& v, spatialaggregate::OcTreeNode< CoordType, ValueType >* current, void* data ), const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	if( type == OCTREE_LEAF_NODE ) {

		if( inRegion( minPosition, maxPosition ) ) {
			f( value, this, data );
		}

	}
	else {

		if( !overlap( minPosition, maxPosition ) )
			return;

		if( containedInRegion( minPosition, maxPosition ) ) {
			f( value, this, data );
			return;
		}

		if( (this->maxPosition[0] - this->minPosition[0]) - minimumSearchVolumeSize <= -OCTREE_EPSILON * minimumSearchVolumeSize ) {
			// since we check for overlap above, this branching node is only accounted for, when its extent overlaps with the search region
			f( value, this, data );
			return;
		}

		for( unsigned int i = 0; i < 8; i++ ) {
			if(!siblings[i])
				continue;

			siblings[i]->applyOperatorInVolume( value, data, f, minPosition, maxPosition, minimumSearchVolumeSize );

		}

	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::collectNodesInDepthRange( std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* >& nodes, unsigned int minDepth, unsigned int maxDepth ) {

	if( type == OCTREE_LEAF_NODE ) {

		if( depth >= minDepth && depth <= maxDepth )
			nodes.push_back( this );

	}
	else {

		if( depth >= minDepth && depth <= maxDepth ) {
			nodes.push_back( this );
		}

		for( unsigned int i = 0; i < 8; i++ ) {
			if( !siblings[i] )
				continue;
			siblings[i]->collectNodesInDepthRange( nodes, minDepth, maxDepth );
		}

	}

}


template< typename CoordType, typename ValueType >
inline spatialaggregate::OcTreeNode< CoordType, ValueType >* spatialaggregate::OcTreeNode< CoordType, ValueType >::getTightestNode( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	if( type == OCTREE_LEAF_NODE ) {

		if( regionContained( minPosition, maxPosition ) ) {
			return this;
		}
		else {
			// check parent
			return parent->getTightestNode( minPosition, maxPosition, minimumSearchVolumeSize );
		}

	}
	else {

		if( regionContained( minPosition, maxPosition ) ) {

			if( (this->maxPosition[0] - this->minPosition[0]) - minimumSearchVolumeSize <= -OCTREE_EPSILON * minimumSearchVolumeSize )
				return this;

			// check if siblings are tighter
			for( unsigned int i = 0; i < 8; i++ ) {
				if( !siblings[i] )
					continue;

				if( siblings[i]->regionContained( minPosition, maxPosition ) ) {
					return siblings[i]->getTightestNode( minPosition, maxPosition, minimumSearchVolumeSize );
				}
			}
			// no tighter sibling found
			return this;
		}
		else {
			// check if parent is tighter
			if( !parent )
				return this;
			return parent->getTightestNode( minPosition, maxPosition, minimumSearchVolumeSize );
		}

		return this;

	}

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTreeNode< CoordType, ValueType >::sweepDown( void* data, void (*f)( spatialaggregate::OcTreeNode< CoordType, ValueType >* current, spatialaggregate::OcTreeNode< CoordType, ValueType >* next, void* data ) ) {

	if( type == OCTREE_LEAF_NODE ) {

		f( this, NULL, data );

	}
	else {

		for( unsigned int i = 0; i < 8; i++ ) {
			f( this, siblings[i], data );
			if( siblings[i] )
				siblings[i]->sweepDown( data, f );
		}

	}

}


template< typename CoordType, typename ValueType >
inline spatialaggregate::OcTreeNode< CoordType, ValueType >* spatialaggregate::OcTreeNode< CoordType, ValueType >::addPoint( const spatialaggregate::OcTreePoint< CoordType, ValueType >& point, CoordType minimumVolumeSize ) {

	spatialaggregate::OcTreeNode< CoordType, ValueType >* n = allocator->allocateLeafNode();//new spatialaggregate::OcTreeLeafNode< CoordType, ValueType >();
	n->position = point.position;
	n->closestPosition = point.position;
	n->closestPositionDistance = 0;
	n->value = point.value;
	n->numPoints = 1;
	return addPoint( n, minimumVolumeSize );

}

template< typename CoordType, typename ValueType >
inline spatialaggregate::OcTreeNode< CoordType, ValueType >* spatialaggregate::OcTreeNode< CoordType, ValueType >::addPoint( spatialaggregate::OcTreeNode< CoordType, ValueType >* leaf, CoordType minimumVolumeSize ) {

	// traverse from root until we found an empty leaf node
	spatialaggregate::OcTreeNode< CoordType, ValueType >* parentNode = parent;
	unsigned int octant = 0;
	spatialaggregate::OcTreeNode< CoordType, ValueType >* currNode = this;

	while( currNode ) {

		if( currNode->type == OCTREE_LEAF_NODE ) {
			break; // reached a leaf, stop searching
		}

		parentNode = currNode;
		octant = currNode->getOctant( leaf->position );
		currNode = currNode->siblings[ octant ];

		// add value to integral value of parent node
		parentNode->value += leaf->value;

		// count successors of parent
		parentNode->numPoints += leaf->numPoints;

		// check if position is more central to the parentNode's position
		CoordType dx = parentNode->position[0] - leaf->position[0];
		CoordType dy = parentNode->position[1] - leaf->position[1];
		CoordType dz = parentNode->position[2] - leaf->position[2];
		CoordType dist2 = dx*dx+dy*dy+dz*dz;

		if( dist2 < parentNode->closestPositionDistance ) {
			parentNode->closestPosition[0] = leaf->position[0];
			parentNode->closestPosition[1] = leaf->position[1];
			parentNode->closestPosition[2] = leaf->position[2];
			parentNode->closestPositionDistance = dist2;
		}

	}


	if( currNode == NULL ) {

		// simply add a new leaf node in the parent's octant
		leaf->parent = parentNode;
		leaf->setDimensionsForParentOctant( octant ); // to set min max positions
		leaf->depth = parentNode->depth + 1;

		parentNode->siblings[octant] = leaf;

		return leaf;

	}
	else {

		// branch at parent's octant..
		spatialaggregate::OcTreeNode< CoordType, ValueType >* oldLeaf = currNode;// ((spatialaggregate::OcTreeBranchingNode< CoordType, ValueType >*) parentNode)->siblings[octant]);


		// check if old leaf covers less than double the minimumVolumeSize
		if( (oldLeaf->maxPosition[0] - oldLeaf->minPosition[0]) - minimumVolumeSize * 2.f <= -OCTREE_EPSILON * minimumVolumeSize ) {

			// discard point
			// TODO: average point position?
			oldLeaf->value += leaf->value;
			oldLeaf->numPoints += leaf->numPoints;
			allocator->deallocate( leaf );
			return oldLeaf;

		}


		// just skip double points
		if( oldLeaf->position == leaf->position ) {
			oldLeaf->value += leaf->value;
			oldLeaf->numPoints += leaf->numPoints;
			allocator->deallocate( leaf );
			return oldLeaf;
		}

		spatialaggregate::OcTreeNode< CoordType, ValueType >* branch = allocator->allocateBranchingNode();//new spatialaggregate::OcTreeBranchingNode< CoordType, ValueType >();
		branch->parent = parentNode;
		branch->setDimensionsForParentOctant( octant );
		branch->position = (branch->minPosition + branch->maxPosition) * 0.5f;
		branch->depth = parentNode->depth + 1;

		// initialize closest position
		CoordType dx = branch->position[0] - oldLeaf->position[0];
		CoordType dy = branch->position[1] - oldLeaf->position[1];
		CoordType dz = branch->position[2] - oldLeaf->position[2];

		branch->closestPosition[0] = oldLeaf->position[0];
		branch->closestPosition[1] = oldLeaf->position[1];
		branch->closestPosition[2] = oldLeaf->position[2];
		branch->closestPositionDistance = dx*dx+dy*dy+dz*dz;

		parentNode->siblings[octant] = branch;


		branch->addPoint( oldLeaf, minimumVolumeSize );

		// js: to initialize values that are not integrated in ValueType!
		branch->value = oldLeaf->value;
		branch->numPoints = oldLeaf->numPoints;

		return branch->addPoint( leaf, minimumVolumeSize ); // this could return some older leaf

	}

}


template< typename CoordType, typename ValueType >
spatialaggregate::OcTree< CoordType, ValueType >::OcTree( const spatialaggregate::OcTreePosition< CoordType >& dimensions, const spatialaggregate::OcTreePosition< CoordType >& center, CoordType minVolSize, spatialaggregate::OcTreeNodeAllocator< CoordType, ValueType >* allocator ) {

	if( allocator ) {
		this->allocator = allocator;
		deleteAllocator = false;
	}
	else {
		this->allocator = new spatialaggregate::OcTreeNodeAllocator< CoordType, ValueType >();
		deleteAllocator = true;
	}

	initialize( dimensions, center, minVolSize );

}


template< typename CoordType, typename ValueType >
void spatialaggregate::OcTree< CoordType, ValueType >::initialize( const spatialaggregate::OcTreePosition< CoordType >& dimensions, const spatialaggregate::OcTreePosition< CoordType >& center, CoordType minVolSize ) {

	root = allocator->allocateBranchingNode();//new spatialaggregate::OcTreeBranchingNode< CoordType, ValueType >();

	root->parent = NULL;
	root->position = center;
	root->minPosition = center - (dimensions*0.5f);
	root->maxPosition = center + (dimensions*0.5f);

	minimumVolumeSize = minVolSize;

}


template< typename CoordType, typename ValueType >
spatialaggregate::OcTree< CoordType, ValueType >::OcTree( const spatialaggregate::OcTreePosition< CoordType >& center, CoordType minimumVolumeSize, CoordType maxDistance, spatialaggregate::OcTreeNodeAllocator< CoordType, ValueType >* allocator ) {

	if( allocator ) {
		this->allocator = allocator;
		deleteAllocator = false;
	}
	else {
		this->allocator = new spatialaggregate::OcTreeNodeAllocator< CoordType, ValueType >();
		deleteAllocator = true;
	}

	spatialaggregate::OcTreePosition< CoordType > dimensions;

	// determine dimensions to cover maxDistance with 2^k * minimumVolumeSize..
	const float size = minimumVolumeSize * pow( 2.f, ceil( log( 2.f * maxDistance / minimumVolumeSize ) / log( 2.f ) ) );

	dimensions[0] = size;
	dimensions[1] = size;
	dimensions[2] = size;

	initialize( dimensions, center, minimumVolumeSize );

}



template< typename CoordType, typename ValueType >
spatialaggregate::OcTree< CoordType, ValueType >::~OcTree() {

	if( root ) {
		allocator->deallocate( root );
	}

	if( deleteAllocator )
		delete allocator;
}



template< typename CoordType, typename ValueType >
inline spatialaggregate::OcTreeNode< CoordType, ValueType >* spatialaggregate::OcTree< CoordType, ValueType >::addPoint( const OcTreePoint< CoordType, ValueType >& point ) {

	return root->addPoint( point, minimumVolumeSize );

}


template< typename CoordType, typename ValueType >
inline spatialaggregate::OcTreeNode< CoordType, ValueType >* spatialaggregate::OcTree< CoordType, ValueType >::getClosestPoint( const spatialaggregate::OcTreePosition< CoordType >& position ) {

	// TODO
	return NULL;

}



template< typename CoordType, typename ValueType >
inline std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* > spatialaggregate::OcTree< CoordType, ValueType >::getPointsInVolume( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* > points;

	root->getPointsInVolume( points, minPosition, maxPosition, minimumSearchVolumeSize );

	return points;

}

template< typename CoordType, typename ValueType >
inline std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* > spatialaggregate::OcTree< CoordType, ValueType >::getAllNodesInVolume( const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* > points;

	root->getAllNodesInVolume( points, minPosition, maxPosition, minimumSearchVolumeSize );

	return points;

}


template< typename CoordType, typename ValueType >
inline ValueType spatialaggregate::OcTree< CoordType, ValueType >::getValueInVolume( const OcTreePosition< CoordType >& minPosition, const OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	return root->getValueInVolume( minPosition, maxPosition, minimumSearchVolumeSize );

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTree< CoordType, ValueType >::getValueAndCountInVolume( ValueType& value, unsigned int& count, const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	value = ValueType(0);
	count = 0;
	root->getValueAndCountInVolume( value, count, minPosition, maxPosition, minimumSearchVolumeSize );

}


template< typename CoordType, typename ValueType >
inline void spatialaggregate::OcTree< CoordType, ValueType >::applyOperatorInVolume( ValueType& value, void* data, void (*f)( ValueType& v, OcTreeNode< CoordType, ValueType >* current, void* data ), const spatialaggregate::OcTreePosition< CoordType >& minPosition, const spatialaggregate::OcTreePosition< CoordType >& maxPosition, CoordType minimumSearchVolumeSize ) {

	root->applyOperatorInVolume( value, data, f, minPosition, maxPosition, minimumSearchVolumeSize );

}


