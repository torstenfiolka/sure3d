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

#ifndef SURE_MAP2D_H_
#define SURE_MAP2D_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace sure
{
  namespace data
  {

    /**
     * 2D data structure with information wether a position contains information or not
     */
    template<typename Type>
    class Map2d
    {
      public:

        Map2d();
        Map2d(Type defaultValue);
        Map2d(unsigned int width, unsigned int height);
        Map2d(unsigned int width, unsigned int height, Type defaultValue);
        Map2d(const Map2d<Type>& obj);
        ~Map2d() { reset(); }

        void reset();
        void clear();
        void resize(unsigned int width, unsigned int height);

        Map2d<Type>& operator=(const Map2d<Type>& rValue);

        unsigned int getX(unsigned int i) const { return (i % width); }
        unsigned int getY(unsigned int i) const { return (i / width); }

        Type at(unsigned int index) const;
        Type& at(unsigned int index);
        Type at(unsigned int x, unsigned int y) const { return at(y*width+x); }
        Type& at(unsigned int x, unsigned int y) { return at(y*width+x); }

        bool exists(unsigned int index) const;
        bool& exists(unsigned int index);
        bool exists(unsigned int x, unsigned int y) const { return exists(y*width+x); }
        bool& exists(unsigned int x, unsigned int y) { return exists(y*width+x); }

        void set(unsigned int index, const Type& t);
        void set(unsigned int x, unsigned int y, const Type& t) { set(y*width+x, t); }

        void remove(unsigned int index);
        void remove(unsigned int x, unsigned int y) { remove(y*width+x); }

        void removeRow(unsigned int row);
        void removeColumn(unsigned int column);

        std::vector<Type> getVector(bool onlyValid = true) const;
        Map2d<Type> subsample(int step) const;

        Type* map;
        bool* valid;

        Type defaultValue, invalidValue;
        bool invalidStatus;

        unsigned int width, height, size;

    };

  } // namespace
} // namespace

#include "sure/data/impl/map2d.hpp"

#endif /* MAP2D_H_ */
