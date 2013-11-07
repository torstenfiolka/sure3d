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

#ifndef SURE_RANGE_IMAGE_H_
#define SURE_RANGE_IMAGE_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>

#include <sure/data/typedef.h>
#include <sure/data/map2d.h>

namespace sure
{
  namespace range_image
  {

    typedef sure::data::Map2d<Scalar> Map2d;

    /*
     * Range image implementation for calulating depth borders
     */
    template <typename PointT>
    class RangeImage : public Map2d, public pcl::PCLBase<PointT>
    {
      public:

      using pcl::PCLBase<PointT>::input_;
      using pcl::PCLBase<PointT>::indices_;
      using pcl::PCLBase<PointT>::initCompute;

      enum Border
      {
        NONE = 0,
        FOREGROUND,
        BACKGROUND,
      };

      static const float DEPTH_JUMP_FACTOR = 1.1f;

      RangeImage() : Map2d(std::numeric_limits<Scalar>::infinity()), borderMap(NONE) { }
      RangeImage(const RangeImage<PointT>& rhs) : Map2d(rhs), pcl::PCLBase<PointT>(rhs), borderMap(rhs.borderMap) { }
      ~RangeImage() { }

      void reset()
      {
        Map2d::reset();
      }

      void calculateRangeImage();

      bool isBackgroundBorder(int index) const { return (borderMap.at(index) == BACKGROUND); }
      bool isForegroundBorder(int index) const { return (borderMap.at(index) == FOREGROUND); }

      void addPointsOnBorders(Scalar stepDist, Scalar maxPointDist, pcl::PointCloud<PointT>& addedPoints);

      Border hasBorder(int index) const { return borderMap.at(index); }
      Border hasBorder(int x, int y) const { return borderMap.at(x, y); }

      protected:

      bool ownsBorder(int x, int y, Scalar& dist);
      float getDensity(int x, int y, int radius = 1);
      float getMeanDensity(int x, int y, int radius = 1);

      void calculateBorderMap();
      Border calculateBorder(int index, Scalar& dist) const;

      sure::data::Map2d<Border> borderMap;

    };

    template <typename PointT>
    pcl::RangeImage createRangeImage(const pcl::PointCloud<PointT>& cloud, Eigen::Affine3f& sensorPose, bool usePCL = false, float angularResolution = 0.09f);

  } // namespace
} // namespace

#include <sure/data/impl/range_image.hpp>

#endif /* SURE_RANGE_IMAGE_H_ */
