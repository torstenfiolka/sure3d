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

#ifndef NORMAL_ESTIMATION_H_
#define NORMAL_ESTIMATION_H_

#include <Eigen/Dense>

#include <sure/memory/fixed_size_allocator_direct_access.h>
#include <sure/normal/normal.h>
#include <sure/normal/normal_histogram.h>
#include <sure/payload/payload_xyzrgb.h>
#include <sure/payload/payload_normal.h>
#include <sure/octree/octree_node.h>
#include <sure/octree/octree.h>

namespace sure
{
  namespace normal
  {
    typedef sure::payload::PointsRGB PointsRGB;
    typedef sure::octree::Node<PointsRGB> Node;
    typedef sure::octree::Octree<PointsRGB> Octree;
    typedef Octree::NodeVector NodeVector;
    typedef sure::payload::NormalPayload NormalPayload;
    typedef sure::payload::PointsRGB FixedPayload;
    typedef sure::PointFlag PointFlag;

    /**
     * Allocates the optional payload for storing normals to all octree nodes with a given edge length
     * @param octree
     * @param samplingrate Defines the edge length of the octree nodes
     * @param allocator The allocator which allocates and owns the normal payload
     */
    void allocateNormalPayload(Octree& octree, Scalar samplingrate, sure::memory::FixedSizeAllocatorWithDirectAccess<NormalPayload>& allocator);

    /**
     * Orientates a given normal toward a given orientation point, usually the viewpoint
     * @param pos Position of the normal
     * @param normal
     * @param orientation Position towards the normal will be orientated
     */
    void orientateNormal(const Vector3& pos, Normal& normal, const Vector3& orientation);

    /**
     * Estimates a normal on a given position with a given radius around it
     * @param octree
     * @param p The position for the normal
     * @param radius The radius defining a box around the position for integrating point information
     * @param normal The resulting normal
     * @return True, if a stable normal was estimated, false otherwise
     */
    bool estimateNormal(const Octree& octree, const Vector3& p, Scalar radius, Normal& normal);

    /**
     * Estimated normals in the octree on all nodes with an edge length corresponding to the samplingrate
     * @param octree
     * @param samplingrate
     * @param radius Radius of the box around a designated normal position in which all point information will be integrated
     * @param orientationPoint Any normal will be orientated towards this point
     * @param histogramInfluence Determines the range on the unit sphere's surface a normal will influence the underlying histogram
     */
    unsigned estimateNormals(Octree& octree, Scalar samplingrate, Scalar radius, const Vector3& orientationPoint, Scalar histogramInfluence);

    /**
     * Sets normals from nodes with a given flag as invalid
     * @param octree
     * @param samplingrate
     * @param flag
     */
    unsigned discardNormalsfromNodesWithFlag(Octree& octree, Scalar samplingrate, PointFlag flag);

  }
}

#endif /* NORMAL_ESTIMATION_H_ */
