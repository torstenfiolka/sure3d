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

#ifndef __DOWNSAMPLE_OCTREE_H__
#define __DOWNSAMPLE_OCTREE_H__

#include <map>
#include <vector>

#include <sure/octreelib/spatialaggregate/octree.h>

namespace algorithm {
	
	template< typename CoordType, typename ValueType >
	class OcTreeSamplingMap : public std::map< unsigned int, std::vector< spatialaggregate::OcTreeNode< CoordType, ValueType >* > > {
	public:
		OcTreeSamplingMap() {}
		~OcTreeSamplingMap() {}
	};
	
	// downToDepth: include a leaf at depth in lists from 0 to depth
	template< typename CoordType, typename ValueType >
	OcTreeSamplingMap< CoordType, ValueType > downsampleOcTree( const spatialaggregate::OcTree< CoordType, ValueType >& tree, bool downToMaxDepth = false, unsigned int maxDepth = 0, unsigned int numPoints = 0 );
	
};

#include <sure/octreelib/algorithm/downsample.hpp>

#endif //__DOWNSAMPLE_OCTREE_H__


