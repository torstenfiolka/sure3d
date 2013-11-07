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

#ifndef SURE_OCTREE_NODE_H_
#define SURE_OCTREE_NODE_H_

#include <cstring>

#include <sure/access/region.h>
#include <sure/payload/payload.h>

#include <sure/payload/payload_xyzrgb.h>
#include <sure/payload/payload_normal.h>
#include <sure/payload/payload_cross_product.h>
#include <sure/payload/payload_entropy.h>

namespace sure
{
  namespace octree
  {

    typedef sure::PointUnit PointUnit;
    typedef sure::access::Point Point;
    typedef sure::access::Region Region;
    typedef sure::OctantType OctantType;
    typedef sure::payload::Payload OptionalPayload;

    template <typename FixedPayloadT>
    class Octree;
    template <typename FixedPayloadT>
    class Node;

    template <typename FixedPayloadT>
    std::ostream& operator<<(std::ostream& stream, Node<FixedPayloadT>& rhs);

    /**
     * Octree node
     * The FixedPayloadT must implement the += operator
     */
    template <typename FixedPayloadT>
    class Node
    {
      public:

        Node() : region_(), fixed_(), opt_(NULL), parent_(NULL)
        {
          clearChildren();
        }

        Node(const Region& r) : region_(r), fixed_(), opt_(NULL), parent_(NULL)
        {
          clearChildren();
        }

        //! Parents or children will NOT be copied
        Node(const Node& rhs) : region_(rhs.region_), fixed_(rhs.fixed_), opt_(NULL), parent_(NULL)
        {
          clearChildren();
        }

        ~Node() { }

        Node operator+(const Node& rhs) const
        {
          Node r(*this);
          r.fixed_ += rhs.fixed_;
          return r;
        }

        Node& operator+=(const Node& rhs)
        {
          this->fixed_ += rhs.fixed_;
          return *this;
        }

        //! Only compares the regions
        bool operator==(const Node& rhs) const
        {
          return (this->region_ == rhs.region_);
        }
        bool operator==(const Region& rhs) const
        {
          return (this->region_ == rhs);
        }

        //! Returns the region covered by the node
        const Region& region() const { return region_; }
        //! Returns the center of the nodes' region
        Point center() const { return region_.center(); }

        //! Access to the fixed payload
        const FixedPayloadT& fixed() const { return fixed_; }
        FixedPayloadT& fixed() { return fixed_; }

        //! Acces to the optional payload
        OptionalPayload* opt() { return opt_; }
        const OptionalPayload* opt() const { return opt_; }

        /**
         * Sets the optional payload. Note: The node never takes ownership of the pointer,
         * you are required to handle the deletion of the object
         * @param o
         */
        void setOptionalPayload(OptionalPayload* o) { opt_ = o; }

        //! Returns the depth of the node
        unsigned depth() const { return (parent_ ? parent_->depth()+1 : 0); }

      protected:

        void clearChildren()
        {
          for(OctantType i=0; i<OCTANT; ++i)
          {
            children_[i] = NULL;
          }
        }

        Region region_;

        FixedPayloadT fixed_;
        OptionalPayload* opt_;

        Node* children_[OCTANT];
        Node* parent_;

      private:

        friend class Octree<FixedPayloadT>;

        friend std::ostream& operator<< <>(std::ostream& stream, Node& rhs);

    };

  } // namespace
} // namespace

#include <sure/octree/impl/octree_node.hpp>

#endif /* SURE_OCTREE_NODE_H_ */
