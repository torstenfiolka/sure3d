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

// Calculates range image and depth border
template <typename PointT>
void sure::range_image::RangeImage<PointT>::calculateRangeImage()
{
  if( input_->empty() || input_->width == 1 || input_->height == 1 )
  {
    return;
  }
  initCompute();
  if( this->width != input_->width || this->height != input_->height )
  {
    this->resize(input_->width, input_->height);
  }
  else
  {
    this->clear();
  }

//  std::cerr << "Range Image " << width << "x" << height << std::endl;

  float dist = 0.f;
  Vector3 viewVector = Vector3::Zero();

  for(int i=0; i<(int) indices_->size(); ++i)
  {
    const PointT& p = input_->points[indices_->at(i)];
    if( std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) )
    {
      viewVector[0] = p.x - input_->sensor_origin_(0);
      viewVector[1] = p.y - input_->sensor_origin_(1);
      viewVector[2] = p.z - input_->sensor_origin_(2);
      dist = std::min(viewVector.norm(), MAX_USED_POINT_DISTANCE);
    }
    else
    {
      dist = INFINITY;
    }
    this->at(indices_->at(i)) = dist;
    this->exists(indices_->at(i)) = true;
  }

  calculateBorderMap();
//  std::cerr << "BorderMap " << borderMap.width << "x" << borderMap.height << std::endl;
}

template <typename PointT>
typename sure::range_image::RangeImage<PointT>::Border sure::range_image::RangeImage<PointT>::calculateBorder(int index, Scalar& dist) const
{
  const int x = index % this->width, y = index / this->width;
  float minDist = MAX_USED_POINT_DISTANCE, maxDist = 0.f;
  const float& referenceDistance = this->at(x, y);
  sure::range_image::RangeImage<PointT>::Border ret = sure::range_image::RangeImage<PointT>::NONE;
  dist = 0.0;

  if( !this->exists(x, y) || isinf(referenceDistance) || referenceDistance == MAX_USED_POINT_DISTANCE )
  {
    dist = MAX_USED_POINT_DISTANCE;
    return sure::range_image::RangeImage<PointT>::BACKGROUND;
  }

  for(int i=1; true; ++i)
  {
    if( x+i >= (int) width-1 )
    {
      dist = MAX_USED_POINT_DISTANCE;
      return sure::range_image::RangeImage<PointT>::BACKGROUND;
    }
    const float& currDist = this->at(x+i, y);
    if( !isinf(currDist) )
    {
      maxDist = std::max(currDist, maxDist);
      minDist = std::min(currDist, minDist);
      break;
    }
  }

  for(int i=1; true; ++i)
  {
    if( x-i <= 0 )
    {
      dist = MAX_USED_POINT_DISTANCE;
      return sure::range_image::RangeImage<PointT>::BACKGROUND;
    }
    const float& currDist = this->at(x-i, y);
    if( !isinf(currDist) )
    {
      maxDist = std::max(currDist, maxDist);
      minDist = std::min(currDist, minDist);
      break;
    }
  }

  for(int i=1; true; ++i)
  {
    if( y+i >= (int) height-1 )
    {
      dist = MAX_USED_POINT_DISTANCE;
      return sure::range_image::RangeImage<PointT>::BACKGROUND;
    }
    const float& currDist = this->at(x, y+i);
    if( !isinf(currDist) )
    {
      maxDist = std::max(currDist, maxDist);
      minDist = std::min(currDist, minDist);
      break;
    }
  }

  for(int i=1; true; ++i)
  {
    if( y-i <= 0 )
    {
      dist = MAX_USED_POINT_DISTANCE;
      return sure::range_image::RangeImage<PointT>::BACKGROUND;
    }
    const float& currDist = this->at(x, y-i);
    if( !isinf(currDist) )
    {
      maxDist = std::max(currDist, maxDist);
      minDist = std::min(currDist, minDist);
      break;
    }
  }

  if( maxDist > referenceDistance*DEPTH_JUMP_FACTOR )
  {
    ret = sure::range_image::RangeImage<PointT>::FOREGROUND;
    dist = maxDist - referenceDistance;
  }
  if( minDist < referenceDistance/DEPTH_JUMP_FACTOR )
  {
    ret = sure::range_image::RangeImage<PointT>::BACKGROUND;
    dist = referenceDistance - minDist;
  }
  return ret;
}

/**
 * Adds points on foreground depth borders along the view direction
 * @param stepDist distance between two added points
 * @param maxPointDist maximum allowed distance from originating point
 * @param addedPoints contains the added points
 */
template <typename PointT>
void sure::range_image::RangeImage<PointT>::addPointsOnBorders(Scalar stepDist, Scalar maxPointDist, pcl::PointCloud<PointT>& addedPoints)
{
  Vector3 viewPoint(input_->sensor_origin_[0], input_->sensor_origin_[1], input_->sensor_origin_[2]);
  addedPoints.clear();
  addedPoints.header = input_->header;

  for(unsigned int i=0; i<this->size; ++i)
  {
    Scalar maxDist;
    if( calculateBorder(i, maxDist) == sure::range_image::RangeImage<PointT>::FOREGROUND )
    {
      maxDist = std::min(maxDist, maxPointDist);

      Vector3 startPoint(input_->points[i].x, input_->points[i].y, input_->points[i].z);
      Vector3 viewDirection(startPoint - viewPoint);
      float startPointColor = input_->points[i].rgb;
      float distanceDone = 0.f;

      while( distanceDone < maxDist )
      {
        PointT p;
        distanceDone += stepDist;
        Vector3 newPoint = startPoint + (viewDirection*distanceDone);
        p.x = newPoint[0];
        p.y = newPoint[1];
        p.z = newPoint[2];
        p.rgb = startPointColor;
        addedPoints.push_back(p);
      }
    }
  }
}

template <typename PointT>
void sure::range_image::RangeImage<PointT>::calculateBorderMap()
{
  Scalar dummy;

  if( borderMap.width != input_->width || borderMap.height != input_->height )
  {
    borderMap.resize(input_->width, input_->height);
  }
  else
  {
    borderMap.clear();
  }

  for(unsigned int i=0; i<indices_->size(); ++i)
  {
      borderMap.set(i, calculateBorder(i, dummy));
//    if( !isinf(this->at(i)) && this->at(i) < sure::MAX_USED_POINT_DISTANCE )
//    {
//    }
//    else
//    {
//      borderMap.set(i, sure::range_image::RangeImage<PointT>::NONE);
//    }
  }
}

template <typename PointT>
pcl::RangeImage sure::range_image::createRangeImage(const pcl::PointCloud<PointT>& cloud, Eigen::Affine3f& sensorPose, bool usePCL, float angularResolution)
{
  pcl::RangeImage rangeImage;
  if( cloud.width == 1 || cloud.height == 1 || usePCL )
  {
    float maxAngleWidth     = 360.0f * (M_PI/180.0f); // 360.0 degree in radv
    float maxAngleHeight    = 180.0f * (M_PI/180.0f); // 180.0 degree in rad
    angularResolution *= (M_PI / 180.f);
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.f;
    float minRange = 0.5f;
    int borderSize = 1;
    rangeImage.reset();
    rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);
//    ROS_DEBUG_NAMED("io", "Finished creating a %ix%i range image using PCL with %i/%i points.", rangeImage.width, rangeImage.height, (int) rangeImage.points.size(), (int) cloud.points.size());
  }
  else
  {
    Eigen::Vector3f viewPoint = (sensorPose.rotation()*Eigen::Vector3f::Zero() + sensorPose.translation());
    Eigen::Vector3f rangePoint;
    rangeImage.height = cloud.height;
    rangeImage.width = cloud.width;
    rangeImage.header = cloud.header;
    rangeImage.is_dense = cloud.is_dense;
    rangeImage.points.resize(cloud.points.size());
    for(int i=0; i<(int) cloud.points.size(); ++i)
    {
      rangePoint = Vector3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      rangeImage.points[i].x = cloud.points[i].x;
      rangeImage.points[i].y = cloud.points[i].y;
      rangeImage.points[i].z = cloud.points[i].z;
      rangeImage.points[i].range = (rangePoint - viewPoint).norm();
    }
    rangeImage.sensor_origin_[0] = viewPoint[0];
    rangeImage.sensor_origin_[1] = viewPoint[1];
    rangeImage.sensor_origin_[2] = viewPoint[2];
    rangeImage.sensor_origin_[3] = 0.f;
    rangeImage.setAngularResolution(0.09f * (M_PI/180.0f));
//    ROS_DEBUG_NAMED("io", "Finished creating a %ix%i range image manually with %i/%i points.", rangeImage.width, rangeImage.height, (int) rangeImage.points.size(), (int) cloud.points.size());
  }
  return rangeImage;
}

template <typename PointT>
float sure::range_image::RangeImage<PointT>::getDensity(int x, int y, int radius)
{
  Vector3 refVec, currVec;
  int index, count = 0;
  float dist = 0.f, refDist = this->at(x, y), currDist;

  if( !this->exists(x, y) || refDist < 0.f || isinf(refDist) )
  {
    return INFINITY;
  }

  for(int i=x-1; i<=x+1; ++i)
  {
    for(int j=y-1; j<=y+1; ++j)
    {
      if( j == y && x == i )
      {
        continue;
      }
      if( this->exists(i, j) )
      {
        currDist = this->at(i, j);
        if( isinf(currDist) || currDist < 0.f )
        {
          continue;
        }
        index = j*width+i;
        dist += fabs(currDist - refDist);
        count++;
      }
    }
  }
  if( count > 0 )
  {
    dist = dist / float(count);
  }
  else
  {
    return INFINITY;
  }
  return dist;
}

template <typename PointT>
float sure::range_image::RangeImage<PointT>::getMeanDensity(int x, int y, int radius)
{
  int count = 0;
  float currDensity, density = 0.f;
  for(int i=x-radius; i<=x+radius; ++i)
  {
    for(int j=y-radius; j<=y+radius; ++j)
    {
      currDensity = getDensity(i, j);
      if( !isinf(currDensity) )
      {
        count++;
        density += currDensity;
      }
    }
  }
  if( count > 0 )
  {
    density = density / float(count);
  }
  else
  {
    return INFINITY;
  }
  return density;
}
