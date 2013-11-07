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

#ifndef SURE_FEATURE_H_
#define SURE_FEATURE_H_

#include <Eigen/Dense>

#include <sure/data/typedef.h>
#include <sure/payload/payload_xyzrgb.h>
#include <sure/payload/payload_normal.h>
#include <sure/octree/octree_node.h>
#include <sure/octree/octree.h>
#include <sure/normal/normal.h>
#include <sure/normal/normal_estimation.h>
#include <sure/descriptor/descriptor.h>

namespace sure
{
  namespace feature
  {

    typedef sure::payload::PointsRGB FixedPayload;
    typedef sure::octree::Node<FixedPayload> Node;
    typedef sure::payload::NormalPayload NormalPayload;
    typedef sure::octree::Octree<FixedPayload> Octree;
    typedef Octree::NodeVector NodeVector;
    typedef sure::descriptor::Descriptor Descriptor;
    typedef sure::normal::Normal Normal;

    /*
     * SURE Feature
     * Contains information about position, orientation and size and optionally a descriptor
     */
    class Feature
    {
      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Feature() : position_(Vector3::Zero()), radius_(0.0), hasDescriptor_(false) { }

        const Descriptor& descriptor (unsigned index) const { return descriptors_.at(index); }
        Descriptor& descriptor(unsigned index) { return descriptors_.at(index); }

        unsigned numberOfDescriptors() const { return descriptors_.size(); }

        bool hasDescriptor() const { return hasDescriptor_; }

        const Normal& normal () const
        {
          return normal_;
        }
        Normal& normal ()
        {
          return normal_;
        }

        const Vector3& position () const
        {
          return position_;
        }
        Vector3& position ()
        {
          return position_;
        }

        Scalar radius() const { return radius_; }
        Scalar& radius() { return radius_; }

        Scalar size() const { return radius_*2.0; }

        void normalizeDescriptor();

        /**
         * Calculates the distance between two feature descriptors. The descriptors must have the same
         * number of distance classes.
         * @param rhs
         * @param shapeWeight Changes the weight of the shape descriptor for the calculation
         * @param colorWeight Changes the weight of the color descriptor for the calculation
         * @param lightnessWeight Changes the weight of the lightness descriptor for the calculation
         * @return
         */
        Scalar distanceTo(const Feature& rhs, Scalar shapeWeight = 1.0, Scalar colorWeight = 1.0, Scalar lightnessWeight = 1.0) const;

        int createDescriptor(const NodeVector& nodes, Scalar referenceLightness, unsigned distanceClasses);

      protected:

        void resetDescriptor(unsigned distanceClasses = sure::feature::DEFAULT_NUMBER_OF_DESCRIPTORS);

        int createShapedescriptor(const NodeVector& nodes);
        int createColordescriptor(const NodeVector& nodes);
        int createLightnessdescriptor(const NodeVector& nodes, Scalar referenceLightness);
        unsigned getDistanceClass(const Vector3& center, const Vector3& pos) const;

        Vector3 position_;
        Normal normal_;
        std::vector<Descriptor> descriptors_;
        Scalar radius_;
        Scalar distanceClassSize_;
        bool hasDescriptor_;

      private:

        friend std::ostream& operator<<(std::ostream& stream, const Feature& rhs);

    };

    std::ostream& operator<<(std::ostream& stream, const Feature& rhs);

  } // namespace
} //namespace

#endif /* SURE_FEATURE_H_ */
