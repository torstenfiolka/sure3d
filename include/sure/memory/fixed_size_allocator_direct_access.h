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

#ifndef SURE_MEMORY_FIXED_SIZE_ALLOCATOR_DIRECT_ACCESS_H_
#define SURE_MEMORY_FIXED_SIZE_ALLOCATOR_DIRECT_ACCESS_H_

#include <cstddef>
#include <new>
#include <ostream>

namespace sure
{
  namespace memory
  {
    /**
     * A templated allocator with fixed size. Template Type must have a public default constructor.
     * Allows indexed access to its elements and does not keep track of the elements used.
     * Any aquired memory will be released only due to a resize or deconstruction.
     * Throws bad_alloc if its capacity is reached.
     */
    template<typename T>
    class FixedSizeAllocatorWithDirectAccess
    {

      public:

        FixedSizeAllocatorWithDirectAccess() : array_(NULL), capacity_(0) { }
        FixedSizeAllocatorWithDirectAccess(std::size_t capacity) : array_(NULL), capacity_(0)
        {
          resize(capacity);
        }

        ~FixedSizeAllocatorWithDirectAccess()
        {
          if( array_ )
          {
            delete[] array_;
            array_ = NULL;
          }
        }

        T* allocate(unsigned index) throw (std::exception)
        {
          if( index > capacity_ )
          {
            throw std::bad_alloc();
          }
          return &array_[index];
        }

        void deallocate(T* ptr) { }
        void deallocate(unsigned index) { array_[index] = T(); }

        //! Calls default constructor to all allocated elements. Pointers remain valid
        void clear()
        {
          for(unsigned int i=0; i<capacity_; ++i)
          {
            array_[i] = T();
          }
        }

        /**
         * Resizes the allocator, if its capacity is lower than the new capacity, other clears its elements.
         * May render its pointers invalid.
         */
        void resizeIfSmaller(std::size_t newCapacity)
        {
          if( capacity_ < newCapacity )
          {
            resize(newCapacity);
          }
          else
          {
            clear();
          }
        }

        //! Resizes the allocator. Renders all pointers invalid. May throw a bad_alloc.
        void resize(std::size_t newCapacity)
        {
          if( array_ )
          {
            delete[] array_;
            array_ = NULL;
          }
          array_ = new T[newCapacity+1];
          capacity_ = newCapacity;
        }

        //! Return the capacity
        std::size_t capacity() const { return capacity_; }

      protected:

        T* array_;
        std::size_t capacity_;

      private:

        FixedSizeAllocatorWithDirectAccess(const FixedSizeAllocatorWithDirectAccess& rhs)
        {
          throw std::bad_alloc();
        }

    };

    template<typename T>
    std::ostream& operator<<(std::ostream& stream, const FixedSizeAllocatorWithDirectAccess<T>& rhs)
    {
      return stream << "Fixed size allocator with direct access - capacity: " << rhs.capacity() << "\n";
    }

  }
}




#endif /* SURE_GRID_FIXED_SIZE_ALLOCATOR_H_ */
