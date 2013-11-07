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

#ifndef SURE_REGION_H_
#define SURE_REGION_H_

#include <sure/access/point.h>
#include <algorithm>

namespace sure
{
  namespace access
  {

    /**
     * Specifies a region in 3D.
     * Includes all points that are greater or equal than the minimum point and less than the maximum point.
     * IMPORTANT: Does not contain the maximum point itself!
     */
    class Region
    {
      public:

        Region() : min_(), max_() { }
        Region(const Point& min, const Point& max) : min_(min), max_(max) { }
        Region(const Point& center, unsigned radius) : min_(center-radius), max_(center+radius) { }
        Region(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) : min_(minX, minY, minZ), max_(maxX, maxY, maxZ) { }

        bool operator==(const Region& rhs) const
        {
          return ((Point) min_ == (Point) rhs.min_) && ((Point) max_ == (Point) rhs.max_);
        }
        bool operator!=(const Region& rhs) const
        {
          return !(*this == rhs);
        }

        /**
         * Returns true if the region contains the point rhs
         * Note: Returns false if max_ is equal to rhs
         */
        bool contains(const Point& rhs) const
        {
          return (min_ <= rhs) && (max_ > rhs);
        }

        /**
         * Returns true, if the region contains or is equal to rhs
         */
        bool contains(const Region& rhs) const
        {
          return (min_ <= rhs.min_) && (max_ >= rhs.max_);
        }

        /**
         * Returns true, if the region contains or overlaps with rhs
         */
        bool overlaps(const Region& rhs) const
        {
          return ((this->min_ < rhs.max_) && (this->max_ > rhs.min_));
        }

        /**
         * Returns the size of the region in each dimension
         */
        Point dimensions() const
        {
          return Point(max_.x() - min_.x(), max_.y() - min_.y(), max_.z() - min_.z());
        }

        /**
         * Returns the size of region in the x dimension
         */
        int size() const
        {
          return (max_.x()-min_.x());
        }

        /**
         * Returns the center point
         */
        Point center() const
        {
          return (Point(min_+max_) / 2);
        }

        Point min() const { return min_; }
        Point& min() { return min_; }

        Point max() const { return max_; }
        Point& max() { return max_; }

        /**
         * Returns the octant number in which point lies
         */
        OctantType getOctant(const Point& point) const
        {
          OctantType ret(0);
          Point center(this->center());

          if( center.x() > point.x() )
          {
            ret = ret | (OctantType) 4;
          }
          if( center.y() > point.y() )
          {
            ret = ret | (OctantType) 2;
          }
          if( center.z() > point.z() )
          {
            ret = ret | (OctantType) 1;
          }
          return ret;
        }

        /**
         * Returns the octant for a given octant number
         */
        Region getOctant(OctantType id) const
        {
          Region r;
          Point c(this->center());
          if( id & (OctantType) 4 )
          {
            r.min_.x() = this->min_.x();
            r.max_.x() = c.x();
          }
          else
          {
            r.min_.x() = c.x();
            r.max_.x() = this->max_.x();
          }

          if( id & (OctantType) 2 )
          {
            r.min_.y() = this->min_.y();
            r.max_.y() = c.y();
          }
          else
          {
            r.min_.y() = c.y();
            r.max_.y() = this->max_.y();
          }

          if( id & (OctantType) 1 )
          {
            r.min_.z() = this->min_.z();
            r.max_.z() = c.z();
          }
          else
          {
            r.min_.z() = c.z();
            r.max_.z() = this->max_.z();
          }
          return r;
        }

      protected:

        Point min_, max_;

      private:

        friend std::ostream& operator<<(std::ostream& stream, const Region& rhs);
    };

    std::ostream& operator<<(std::ostream& stream, const Region& rhs);
  }
}


#endif /* SURE_GRID_RANGE_H_ */
