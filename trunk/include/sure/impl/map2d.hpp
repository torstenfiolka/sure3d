// Software License Agreement (BSD License)
//
// Copyright (c) 2012, Fraunhofer FKIE/US
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

#include <sure/map2d.h>

template<typename Type>
sure::Map2d<Type>::Map2d()
{
  defaultValue = Type();
  map = NULL;
  valid = NULL;
  reset();
}

template<typename Type>
sure::Map2d<Type>::Map2d(Type defaultValue)
{
  this->defaultValue = defaultValue;
  map = NULL;
  valid = NULL;
  reset();
}

template<typename Type>
sure::Map2d<Type>::Map2d(unsigned int width, unsigned int height)
{
  defaultValue = Type();
  map = NULL;
  valid = NULL;
  resize(width, height);
}

template<typename Type>
sure::Map2d<Type>::Map2d(unsigned int width, unsigned int height, Type defaultValue)
{
  this->defaultValue = defaultValue;
  map = NULL;
  valid = NULL;
  resize(width, height);
}

template<typename Type>
sure::Map2d<Type>::~Map2d()
{
  reset();
}

template<typename Type>
sure::Map2d<Type>::Map2d(const sure::Map2d<Type>& obj)
{
  this->map = NULL;
  this->valid = NULL;
  this->operator=(obj);
}

template<typename Type>
inline void sure::Map2d<Type>::reset()
{
  width = height = size = 0;
  if( map )
  {
    delete[] map;
    map = NULL;
  }
  if( valid )
  {
    delete[] valid;
    valid = NULL;
  }
}

template<typename Type>
inline void sure::Map2d<Type>::clear()
{
  for(unsigned int i=0; i<size; ++i)
  {
    if( map )
      map[i] = defaultValue;
    if( valid )
      valid[i] = false;
  }
}

template<typename Type>
inline void sure::Map2d<Type>::resize(unsigned int width, unsigned int height)
{
  reset();
  this->width = width;
  this->height = height;
  size = width*height;
  if( size > 0 )
  {
    map = new Type[size];
    valid = new bool[size];
  }
  clear();
}

template<typename Type>
sure::Map2d<Type>& sure::Map2d<Type>::operator=(const sure::Map2d<Type>& rValue)
{
  if( this != &rValue)
  {
    this->resize(rValue.width, rValue.height);
    this->defaultValue = rValue.defaultValue;
    for(unsigned int i=0; i<size; ++i)
    {
      map[i] = rValue.map[i];
      valid[i] = rValue.valid[i];
    }
  }
  return *this;
}

//template<typename Type>
//inline Type sure::Map2d<Type>::at(unsigned int x, unsigned int y) const
//{
//  if( x > width || y > height || !map )
//  {
//    return defaultValue;
//  }
//  return map[y*width + x];
//}
//
//template<typename Type>
//inline Type& sure::Map2d<Type>::at(unsigned int x, unsigned int y)
//{
//  if( x > width || y > height || !map )
//  {
//    invalidValue = defaultValue;
//    return invalidValue;
//  }
//  return map[y*width + x];
//}

template<typename Type>
inline Type sure::Map2d<Type>::at(unsigned int index) const
{
  if( index > size || !map )
  {
    return defaultValue;
  }
  return map[index];
}

template<typename Type>
inline Type& sure::Map2d<Type>::at(unsigned int index)
{
  if( index > size || !map )
  {
    invalidValue = defaultValue;
    return invalidValue;
  }
  return map[index];
}

// Boolsche Map

template<typename Type>
inline bool sure::Map2d<Type>::exists(unsigned int index) const
{
  if( index > size || !valid )
  {
    return false;
  }
  return valid[index];
}

template<typename Type>
inline bool& sure::Map2d<Type>::exists(unsigned int index)
{
  if( index > size || !valid )
  {
    invalidStatus = false;
    return invalidStatus;
  }
  return valid[index];
}

//template<typename Type>
//inline bool sure::Map2d<Type>::exists(unsigned int x, unsigned int y) const
//{
//  if( x > width || y > height || !valid )
//  {
//    return false;
//  }
//  return valid[y*width + x];
//}
//
//template<typename Type>
//inline bool& sure::Map2d<Type>::exists(unsigned int x, unsigned int y)
//{
//  if( x > width || y > height || !valid )
//  {
//    invalidStatus = false;
//    return invalidStatus;
//  }
//  return valid[y*width + x];
//}

template<typename Type>
inline void sure::Map2d<Type>::set(unsigned int index, const Type& t)
{
  if( index < size )
  {
    map[index] = t;
    valid[index] = true;
  }
}

template<typename Type>
inline void sure::Map2d<Type>::remove(unsigned int index)
{
  if( index < size )
  {
    map[index] = defaultValue;
    valid[index] = false;
  }
}

template<typename Type>
inline void sure::Map2d<Type>::removeRow(unsigned int row)
{
  for(unsigned int i=0; i<width; ++i)
  {
    remove(i, row);
  }
}

template<typename Type>
inline void sure::Map2d<Type>::removeColumn(unsigned int column)
{
  for(unsigned int i=0; i<height; ++i)
  {
    remove(column, i);
  }
}

template<typename Type>
std::vector<Type> sure::Map2d<Type>::getVector(bool onlyValid) const
{
  std::vector<Type> newVector;
  if( onlyValid )
  {
    newVector.reserve(this->size);
  }
  else
  {
    newVector.resize(size, this->defaultValue);
  }
  for(int i=0; i<(int) size; ++i)
  {
    if( onlyValid && valid[i] )
    {
      newVector.push_back(map[i]);
    }
    else
    {
      newVector[i] = map[i];
    }
  }
  return newVector;
}

template<typename Type>
sure::Map2d<Type> sure::Map2d<Type>::subsample(int step) const
{
  sure::Map2d<Type> newMap(width/step, height/step, this->defaultValue);
  for(unsigned int x=0, i=0; x<width; x += step, ++i)
  {
    for(unsigned int y=0, j=0; y<height; y += step, ++j)
    {
      if( this->exists(x,y) )
      {
        newMap.set(i, j, this->at(x,y));
      }
    }
  }
  return newMap;
}
