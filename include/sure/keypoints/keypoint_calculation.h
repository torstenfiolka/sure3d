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

#ifndef SURE_KEYPOINT_CALCULATION_H_
#define SURE_KEYPOINT_CALCULATION_H_

#include <pcl/common/eigen.h>

#include <sure/data/typedef.h>
#include <sure/memory/fixed_size_allocator_direct_access.h>
#include <sure/feature/feature.h>
#include <sure/normal/normal.h>
#include <sure/normal/normal_histogram.h>
#include <sure/payload/payload_xyzrgb.h>
#include <sure/payload/payload_entropy.h>
#include <sure/payload/payload_normal.h>
#include <sure/payload/payload_cross_product.h>
#include <sure/octree/octree_node.h>
#include <sure/octree/octree.h>


namespace sure
{
  namespace keypoints
  {
    typedef sure::normal::Normal Normal;
    typedef sure::payload::PointsRGB FixedPayload;
    typedef sure::octree::Node<FixedPayload> Node;
    typedef std::vector<Node* > NodeVector;
    typedef sure::octree::Octree<FixedPayload> Octree;
    typedef sure::payload::NormalPayload NormalPayload;
    typedef sure::payload::NormalPayload CrossProductPayload;
    typedef sure::payload::EntropyPayload EntropyPayload;
    typedef sure::feature::Feature Feature;

    static const unsigned NUMBER_OF_MEAN_SHIFT_ITERATIONS = 3;

    /**
     * Allocates the optional payload for storing entropy information on all nodes with a given edge length
     * @param octree
     * @param samplingrate Defines the edge length of the octree nodes
     * @param allocator The allocator which allocates and owns the entropy payload
     */
    void allocateEntropyPayload(Octree& octree, Scalar samplingrate, sure::memory::FixedSizeAllocatorWithDirectAccess<EntropyPayload>& allocator);

    /**
     * Marks nodes containing ONLY artificial points so no feature will be extracted
     * @param octree
     * @param samplingrate
     */
    void flagArtificialPoints(Octree& octree, Scalar samplingrate);

    /**
     * Marks nodes wich exceed the distance threshold to the sensor
     * @param octree
     * @param samplingrate
     * @param threshold Distance threshold
     * @param sensorPosition Position for defining the distance
     */
    void flagDistantPoints(Octree& octree, Scalar samplingrate, Scalar threshold, const Vector3& sensorPosition);

    /**
     * Marks nodes containing background border points
     * @param octree
     * @param samplingrate
     */
    void flagBackgroundPoints(Octree& octree, Scalar samplingrate);

    /**
     * Resets the node flags for entropy calculation to NOT_CALCULATED, except for flags concerning
     * point structure, e.g. background and artificial flags
     * @param octree
     * @param samplingrate
     */
    void resetFeatureFlags(Octree& octree, Scalar samplingrate);

    /**
     * Calculates the entropy on octree nodes corresponding to the sampling rate
     * @param octree
     * @param samplingrate Defines the octree nodes on which entropy will be calculated
     * @param normalSamplingrate Defines the octree nodes which contain normals
     * @param radius The radius in which normals will be accumulated for entropy calculation, corresponds to the scale
     * @param threshold Minimum entropy for further feature calculation steps
     * @param mode Defines wether normals or cross-products will be used
     * @param weightMethod Weight method for cross-products only
     */
    void calculateEntropy(Octree& octree, Scalar samplingrate, Scalar normalSamplingrate, Scalar radius, Scalar threshold, EntropyCalculationMode mode, Scalar influenceRadius);

    /**
     * Calculates the entropy on corresponding nodes with normals
     * @param octree
     * @param node
     * @param normalSamplingrate
     * @param radius
     * @return
     */
    Scalar calculateEntropyWithNormals(const Octree& octree, Node* node, Scalar normalSamplingrate, Scalar radius, Scalar influenceRadius);

    /**
     * Calculates the entropy on corresponding nodes with crossproducts between the main normal an neighboring normals
     * @param octree
     * @param node
     * @param normalSamplingrate
     * @param radius
     * @param weightMethod
     * @return
     */
    Scalar calculateEntropyWithCrossproducts(const Octree& octree, Node* node, Scalar normalSamplingrate, Scalar radius, Scalar influenceRadius);

    /**
     * Calculates the entropy on corresponding nodes with pairwise crossproducts between neighboring normals
     * @param octree
     * @param node
     * @param normalSamplingrate
     * @param radius
     * @param weightMethod
     * @return
     */
    Scalar calculateEntropyWithCrossproductsPairwise(const Octree& octree, Node* node, Scalar normalSamplingrate, Scalar radius, Scalar influenceRadius);

    /**
     * Calculates the cornerness for a given node
     * @param octree
     * @param node
     * @param radius The radius in which the cornerness will be calculated, usually corresponding to the scale
     * @return
     */
    Scalar calculateCornerness(const Octree& octree, Node* node, Scalar radius);

    /**
     * Calculates the cornerness on all node corresponding to the sampling rate and discarding nodes which miss
     * the minimum cornerness from further feature calculation steps
     * @param octree
     * @param samplingRate
     * @param radius The radius in which the cornerness will be calculated, usually corresponding to the scale
     * @param threshold Minimum cornerness required for further feature calculation steps
     */
    void calculateCornerness(Octree& octree, Scalar samplingRate, Scalar radius, Scalar threshold);

    /**
     * Extracts keypoints from entropy maxima on nodes corresponding to the samplingrate
     * @param octree
     * @param samplingrate
     * @param searchRadius Radius around each possible feature in which no other node contains a higher entropy
     * @param featureRadius Corresponds to the scale
     * @param features Vector containing the keypoints
     */
    unsigned extractKeypoints(Octree& octree, Scalar samplingrate, Scalar searchRadius, Scalar featureRadius, std::vector<Feature>& features, std::vector<Node*>& keypointNodes);

    /**
     * Shifts a given position with mean shift using the entropy for the gradient descent
     * @param octree
     * @param samplingrate Defines the octree nodes containing entropy information
     * @param radius The radius in which the mean shift will be performed, usually corresponds to the scale
     * @param position Position to be shifted
     */
    void improveLocalization(const Octree& octree, Scalar samplingrate, Scalar radius, Vector3& position);

    /**
     * Improves the localization of the given features
     * @param octree
     * @param samplingrate Defines the octree nodes containing entropy information
     * @param radius The radius in which the mean shift will be performed, usually corresponds to the scale
     * @param features
     */
    void improveLocalization(const Octree& octree, Scalar samplingrate, Scalar radius, std::vector<Feature>& features);

    /**
     * Searchs for features which are too close after localization
     * Means: These feature originate from the same region, but were extracted on different positions due to
     * discretization errors
     *
     * @param searchRadius
     * @param features
     */
    int removeRedundantKeypoints(Scalar searchRadius, std::vector<Feature>& features, std::vector<Node*>& keypointNodes);


  } // namespace
} // namespace


#endif /* KEYPOINT_CALCULATION_H_ */
