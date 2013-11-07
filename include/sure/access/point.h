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

#ifndef SURE_POINT_H_
#define SURE_POINT_H_

#include <ostream>
#include <limits>
#include <cmath>

#include <sure/data/typedef.h>

namespace sure
{
  namespace access
  {

    /**
     * Simple 3D point type
     */
    class Point
    {
      public:

        Point()
        {
          coords_[0] = coords_[1] = coords_[2] = ZERO;
        }

        Point(const int x, const int y, const int z)
        {
          coords_[0] = x;
          coords_[1] = y;
          coords_[2] = z;
        }

        bool operator==(const Point& rhs) const
        {
          return (coords_[0] == rhs.coords_[0]) && (coords_[1] == rhs.coords_[1]) && (coords_[2] == rhs.coords_[2]);
        }
        bool operator!=(const Point& rhs) const
        {
          return !(this->operator ==(rhs));
        }

        bool operator<=(const Point& rhs) const
        {
          return (coords_[0] <= rhs.coords_[0]) && (coords_[1] <= rhs.coords_[1]) && (coords_[2] <= rhs.coords_[2]);
        }
        bool operator>=(const Point& rhs) const
        {
          return (coords_[0] >= rhs.coords_[0]) && (coords_[1] >= rhs.coords_[1]) && (coords_[2] >= rhs.coords_[2]);
        }

        bool operator<(const Point& rhs) const
        {
          return (coords_[0] < rhs.coords_[0]) && (coords_[1] < rhs.coords_[1]) && (coords_[2] < rhs.coords_[2]);
        }
        bool operator>(const Point& rhs) const
        {
          return (coords_[0] > rhs.coords_[0]) && (coords_[1] > rhs.coords_[1]) && (coords_[2] > rhs.coords_[2]);
        }

        Point operator-() const
        {
          return Point(-coords_[0], -coords_[1], -coords_[2]);
        }
        Point operator+(const Point& rhs) const
        {
          return Point(coords_[0] + rhs.coords_[0], coords_[1] + rhs.coords_[1], coords_[2] + rhs.coords_[2]);
        }
        Point operator-(const Point& rhs) const
        {
          return Point(coords_[0] - rhs.coords_[0], coords_[1] - rhs.coords_[1], coords_[2] - rhs.coords_[2]);
        }

        Point& operator+=(const Point& rhs)
        {
          this->coords_[0] += rhs.coords_[0];
          this->coords_[1] += rhs.coords_[1];
          this->coords_[2] += rhs.coords_[2];
          return *this;
        }
        Point& operator-=(const Point& rhs)
        {
          this->coords_[0] -= rhs.coords_[0];
          this->coords_[1] -= rhs.coords_[1];
          this->coords_[2] -= rhs.coords_[2];
          return *this;
        }

        Point operator+(int rhs) const
        {
          return Point(coords_[0]+rhs, coords_[1]+rhs, coords_[2]+rhs);
        }
        Point operator-(int rhs) const
        {
          return Point(coords_[0]-rhs, coords_[1]-rhs, coords_[2]-rhs);
        }
        Point operator*(int rhs) const
        {
          return Point(coords_[0]*rhs, coords_[1]*rhs, coords_[2]*rhs);
        }
        Point operator/(int rhs) const
        {
          return Point(coords_[0]/rhs, coords_[1]/rhs, coords_[2]/rhs);
        }

        Point& operator+=(int rhs)
        {
          this->coords_[0] += rhs;
          this->coords_[1] += rhs;
          this->coords_[2] += rhs;
          return *this;
        }
        Point& operator-=(int rhs)
        {
          this->coords_[0] -= rhs;
          this->coords_[1] -= rhs;
          this->coords_[2] -= rhs;
          return *this;
        }
        Point& operator*=(int rhs)
        {
          this->coords_[0] *= rhs;
          this->coords_[1] *= rhs;
          this->coords_[2] *= rhs;
          return *this;
        }
        Point& operator/=(int rhs)
        {
          this->coords_[0] /= rhs;
          this->coords_[1] /= rhs;
          this->coords_[2] /= rhs;
          return *this;
        }

        unsigned squaredNorm() const { return std::abs(coords_[0]) + std::abs(coords_[1]) + std::abs(coords_[2]); }

        PointUnit& operator[](int i) { return coords_[i]; }
        PointUnit operator[](int i) const { return coords_[i]; }

        PointUnit x() const { return coords_[0]; }
        PointUnit& x() { return coords_[0]; }

        PointUnit y() const { return coords_[1]; }
        PointUnit& y() { return coords_[1]; }

        PointUnit z() const { return coords_[2]; }
        PointUnit& z() { return coords_[2]; }

      protected:

        PointUnit coords_[3];

      private:

        friend std::ostream& operator<<(std::ostream& stream, const Point& rhs);

    };

    std::ostream& operator<<(std::ostream& stream, const Point& rhs);

  } // namespace
} //namespace

#endif /* SURE_POINT_H_ */
