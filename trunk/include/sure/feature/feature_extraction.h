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

#ifndef SURE_FEATURE_EXTRACTION_H_
#define SURE_FEATURE_EXTRACTION_H_

#include <sure/payload/payload_xyzrgb.h>
#include <sure/payload/payload_normal.h>
#include <sure/octree/octree_node.h>
#include <sure/octree/octree.h>
#include <sure/normal/normal_estimation.h>
#include <sure/keypoints/keypoint_calculation.h>
#include <sure/descriptor/descriptor.h>
#include <sure/feature/feature.h>

namespace sure
{
  namespace feature
  {

    typedef sure::payload::PointsRGB FixedPayload;
    typedef sure::octree::Node<FixedPayload> Node;
    typedef sure::payload::NormalPayload NormalPayload;
    typedef sure::octree::Octree<FixedPayload> Octree;
    typedef Octree::NodeVector NodeVector;
    typedef sure::descriptor::ShapeDescriptor ShapeDescriptor;
    typedef sure::descriptor::ColorDescriptor ColorDescriptor;
    typedef sure::descriptor::LightnessDescriptor LightnessDescriptor;

    /**
     * Creates a feature on a given position and a given feature normal
     * @param octree
     * @param position The position for the feature
     * @param samplingrate Samplingrate of the octree nodes used for the descriptor calculation, usually the normal samplingrate
     * @param radius Corresponds to the scale of the feature
     * @param normal The features' center normal incorporating all points in its size
     * @param distanceClasses Number of distance classes for the descriptor
     * @return
     */
    Feature createFeature(const Octree& octree, const Vector3& position, Scalar samplingrate, Scalar radius, const sure::normal::Normal& normal, unsigned distanceClasses);

    /**
     * Creates a feature on a given position
     * @param octree
     * @param position The position for the feature
     * @param samplingrate Samplingrate of the octree nodes used for the descriptor calculation, usually the normal samplingrate
     * @param radius Corresponds to the scale of the feature
     * @param viewPoint The point towards the features' normal will be orientated to.
     * @param distanceClasses Number of distance classes for the descriptor
     * @return
     */
    Feature createFeature(const Octree& octree, const Vector3& position, Scalar samplingrate, Scalar radius, const Vector3& viewPoint, unsigned distanceClasses);

    /**
     * Creates the descriptors for a given set of features
     * @param octree
     * @param features
     * @param samplingrate Samplingrate of the octree nodes used for the descriptor calculation, usually the normal samplingrate
     * @param viewPoint The point towards the features' normal will be orientated to.
     * @param distanceClasses Number of distance classes for the descriptor
     */
    unsigned createDescriptors(const Octree& octree, std::vector<Feature>& features, Scalar samplingrate, const Vector3& viewPoint, unsigned distanceClasses);

    /**
     * Creates a descriptor for a given feature. The feature must already contain a normal
     * @param octree
     * @param feature
     * @param samplingrate Samplingrate of the octree nodes used for the descriptor calculation, usually the normal samplingrate
     * @param distanceClasses Number of distance classes for the descriptor
     * @return
     */
    bool createDescriptor(const Octree& octree, Feature& feature, Scalar samplingrate, unsigned distanceClasses);

  } // namespace
} // namespace



#endif /* FEATURE_EXTRACTION_H_ */
