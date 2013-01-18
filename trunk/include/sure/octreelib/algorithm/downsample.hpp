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
 *
 */


#include <list>

template< typename CoordType, typename ValueType >
algorithm::OcTreeSamplingMap< CoordType, ValueType > algorithm::downsampleOcTree( const spatialaggregate::OcTree< CoordType, ValueType >& tree, bool downToMaxDepth, unsigned int maxDepth, unsigned int numPoints  ) {
	
	algorithm::OcTreeSamplingMap< CoordType, ValueType > samplingMap;
	
	for( unsigned int i = 0; i <= maxDepth; i++ )
		samplingMap[i].reserve( numPoints );
	
	std::list< spatialaggregate::OcTreeNode< CoordType, ValueType >* > openList;
	openList.push_back( tree.root );
	
	while( !openList.empty() ) {
		
		spatialaggregate::OcTreeNode< CoordType, ValueType >* node = openList.front();
		
		if( node->type == spatialaggregate::OCTREE_BRANCHING_NODE ) {
			
			samplingMap[ node->depth ].push_back( node );
		
			for( unsigned int i = 0; i < 8; i++ ) {
				if( node->siblings[i] )
					openList.push_back( node->siblings[i] );
			}
			
		}
		else {
			
			// handle leaf node
			
			if( downToMaxDepth ) {
				for( unsigned int i = node->depth; i <= maxDepth; i++ )
					samplingMap[ i ].push_back( node );
			}
			else
				samplingMap[ node->depth ].push_back( node );
			
		}
		
		openList.pop_front();
		
	}
	
	return samplingMap;
	
}
