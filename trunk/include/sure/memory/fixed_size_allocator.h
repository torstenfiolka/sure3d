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

#ifndef SURE_MEMORY_FIXED_SIZE_ALLOCATOR_H_
#define SURE_MEMORY_FIXED_SIZE_ALLOCATOR_H_

#include <cstddef>
#include <new>
#include <ostream>
#include <iostream>

namespace sure
{
  namespace memory
  {

    /**
     * A templated allocator with fixed size. Template Type must have a public default constructor.
     * Any aquired memory will be released only due to a resize or deconstruction.
     * Throws bad_alloc if its capacity is reached.
     */
    template<typename T>
    class FixedSizeAllocator
    {

      public:

        FixedSizeAllocator() : array_(NULL), current_(NULL), size_(0), capacity_(0)
        {
        }

        /**
         * Initializes the Allocator with the given capacity. May throw a bad_alloc.
         */
        FixedSizeAllocator(std::size_t capacity) : array_(NULL), current_(NULL), size_(0), capacity_(0)
        {
          resize(capacity);
        }

        ~FixedSizeAllocator()
        {
          if( array_ )
          {
            delete[] array_;
            array_ = NULL;
          }
        }

        /**
         * Returns a pointer to the template type.
         * May throw if the allocator is not initialized or its capacity is reached.
         */
        T* allocate() throw (std::exception)
        {
          if( !current_ || size_ == capacity_ )
          {
            throw std::bad_alloc();
          }
          size_++;
          return current_++;
        }

        //! Has no effect.
        void deallocate(T* ptr) { }

        //! Calls default constructor to all allocated elements. Pointers remain valid
        void clear()
        {
          for(unsigned int i=0; i<size_; ++i)
          {
            array_[i] = T();
          }
          size_ = 0;
          current_ = &array_[0];
        }

        /**
         * Resizes the allocator, if its capacity is lower than the new capacity, other clears its elements.
         * May render its pointers invalid. May throw a bad_alloc
         */
        void resizeIfSmaller(std::size_t newCapacity) throw (std::exception)
        {
          if(capacity_ < newCapacity)
          {
            resize(newCapacity);
          }
          else
          {
            clear();
          }
        }

        //! Resizes the allocator. Renders all pointers invalid. May throw a bad_alloc.
        void resize(std::size_t newCapacity) throw (std::exception)
        {
          if( array_ )
          {
            delete[] array_;
            array_ = NULL;
          }
          array_ = new T[newCapacity];
          current_ = &array_[0];
          capacity_ = newCapacity;
        }

        //! Return the used elements
        std::size_t size() const { return size_; }

        //! Return the capacity
        std::size_t capacity() const { return capacity_; }

      protected:

        T* array_;
        T* current_;
        std::size_t size_, capacity_;

      private:

        FixedSizeAllocator(const FixedSizeAllocator& rhs)
        {
          throw std::bad_alloc();
        }

    };

    template<typename T>
    std::ostream& operator<<(std::ostream& stream, const FixedSizeAllocator<T>& rhs)
    {
      return stream << "Fixed size allocator - size " << rhs.size() << " - capacity: " << rhs.capacity() << "\n";
    }

  }
}




#endif /* SURE_GRID_FIXED_SIZE_ALLOCATOR_H_ */
