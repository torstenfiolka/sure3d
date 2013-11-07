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

#include <sure/sure.h>

bool sure::SUREFeatureExtractor::calculateSURE()
{
  if( !initCompute() )
  {
    return false;
  }
  if( input_->size() == 0 )
  {
    return false;
  }

  if( verbose )
  {
    std::cout << "Calculating SURE Features\n\n";
    std::cout << config << "\n";
  }

  bool ret(true);

  ret &= buildOctree();

  ret &= calculateNormals();

  ret &= extractKeypoints();

  ret &= extractFeatures();


  return ret;
}

pcl::PointCloud<pcl::InterestPoint>::Ptr sure::SUREFeatureExtractor::getInterestPoints() const
{
  pcl::PointCloud<pcl::InterestPoint>::Ptr points(new pcl::PointCloud<pcl::InterestPoint>);
  for(unsigned i=0; i<features.size(); ++i)
  {
    const Feature& currFeature = features[i];
    sure::payload::EntropyPayload* entropyPayload = dynamic_cast<sure::payload::EntropyPayload*>(keypointNodes_[i]->opt());
    pcl::InterestPoint p;
    p.x = currFeature.position()[0];
    p.y = currFeature.position()[1];
    p.z = currFeature.position()[2];
    p.strength = currFeature.size();
    points->push_back(p);
  }
  return points;
}


bool sure::SUREFeatureExtractor::buildOctree()
{
  bool useRangeImage = config.AdditionalPointsOnDepthBorders || config.IgnoreBackgroundDetections || config.IgnoreNormalsOnBackgroundDepthBorders;
  if( useRangeImage && (input_->height == 1 || input_->width == 1) )
  {
    useRangeImage = false;
    std::cout << "Pointcloud is not organized, cannot use RangeImage.\n";
  }
  addedPoints.clear();
  Vector3 octreeCenter(config.OctreeCenter[0], config.OctreeCenter[1], config.OctreeCenter[2]);

  octree.initialize(config.OctreeSmallestVoxelSize, config.OctreeRootVoxelSize, config.OctreeMaximumNumberOfNodes, octreeCenter);

  pcl::StopWatch watch;

  if( useRangeImage )
  {
    rangeImage.clear();
    rangeImage.setInputCloud(input_);
    rangeImage.calculateRangeImage();
    if( verbose )
    {
      std::cout << std::setprecision(0);
      std::cout.setf(std::ios_base::fixed);
      std::cout << "Range image calculation took " << watch.getTime() << "ms\n";
      watch.reset();
      std::cout << std::setprecision(3);
    }
  }

  try
  {
    if( useRangeImage )
    {
      octree.addPointCloud(*input_, rangeImage);
    }
    else
    {
      octree.addPointCloud(*input_);
    }
    if( useRangeImage && config.AdditionalPointsOnDepthBorders )
    {
      Scalar dist = config.Samplingrate * 2.0;
      Scalar step = config.OctreeSmallestVoxelSize;
      rangeImage.addPointsOnBorders(step, dist, addedPoints);
      octree.addArtificialPointCloud(addedPoints);
    }
  }
  catch(std::exception &e)
  {
    std::cerr << "Building the octree threw an exception: " << e.what() << "\n";
    return false;
  }

  if( verbose )
  {
    std::cout << octree << "\n";
    std::cout << std::setprecision(0);
    std::cout.setf(std::ios_base::fixed);
    std::cout << "Octree calculation took " << watch.getTime() << "ms\n";
    std::cout << std::setprecision(3);
  }

  return true;
}

bool sure::SUREFeatureExtractor::calculateNormals()
{
  Scalar normalSamplingrate = (config.NormalSamplingrate);
  Scalar normalRadius = config.NormalRegionSize * 0.5;
  Vector3 orientationPoint(input_->sensor_origin_[0], input_->sensor_origin_[1], input_->sensor_origin_[2]);

  pcl::StopWatch watch;
  sure::normal::allocateNormalPayload(octree, normalSamplingrate, normalAllocator_);

  if( config.IgnoreNormalsOnBackgroundDepthBorders )
  {
    sure::normal::discardNormalsfromNodesWithFlag(octree, normalSamplingrate, BACKGROUND_BORDER);
  }
  unsigned normals = sure::normal::estimateNormals(octree, normalSamplingrate, normalRadius, orientationPoint, config.NormalInfluenceRadius);

  if( verbose )
  {
    std::cout << "Calculated " << normals << " normals in " << watch.getTime() << "ms\n";
  }
  return true;
}

bool sure::SUREFeatureExtractor::extractKeypoints()
{
  Scalar samplingrate = config.Samplingrate;
  Scalar normalSamplingrate = config.NormalSamplingrate;
  Vector3 sensorPosition(input_->sensor_origin_[0], input_->sensor_origin_[1], input_->sensor_origin_[2]);
  EntropyCalculationMode entropyMode = (EntropyCalculationMode) config.EntropyMode;

  features.clear();
  keypointNodes_.clear();

  pcl::StopWatch watch;

  sure::keypoints::allocateEntropyPayload(octree, samplingrate, entropyAllocator_);

  keypoints::flagArtificialPoints(octree, samplingrate);

  if( config.DistanceThreshold > 0.0 )
  {
    keypoints::flagDistantPoints(octree, samplingrate, config.DistanceThreshold, sensorPosition);
  }
  if( config.IgnoreBackgroundDetections )
  {
    keypoints::flagBackgroundPoints(octree, samplingrate);
  }

  for(unsigned int i=0; i<config.getScales().size(); ++i)
  {
    const Scalar& currentScale = config.getScale(i);
    Scalar radius = currentScale * 0.5;
    Scalar cornernessRadius = radius;
    Scalar suppressionRadius = config.FeatureSuppressionRatio * radius;
    Scalar localizationRadius = radius;

    keypoints::resetFeatureFlags(octree, samplingrate);

    keypoints::calculateEntropy(octree, samplingrate, normalSamplingrate, radius, config.MinimumEntropyThreshold, entropyMode, config.NormalInfluenceRadius);

    if( config.MinimumCornernessThreshold > 0.0 )
    {
      keypoints::calculateCornerness(octree, samplingrate, cornernessRadius, config.MinimumCornernessThreshold);
    }

    unsigned keypoints = keypoints::extractKeypoints(octree, samplingrate, suppressionRadius, radius, features, keypointNodes_);
    if( verbose )
    {
      std::cout << "Calculated " << keypoints << " keypoints with a scale of " << (currentScale * 100.f) << "cm\n";
    }

    if( config.ImproveLocalization )
    {
      keypoints::improveLocalization(octree, samplingrate, localizationRadius, features);
      int redundantKeypoints = keypoints::removeRedundantKeypoints(localizationRadius, features, keypointNodes_);
      if( verbose )
      {
        std::cout << "Removed " << redundantKeypoints << " Features after localization\n";
      }
    }

  }

  if( verbose )
  {
    std::cout << "Calculated " << features.size() << " keypoints on all scales in " << watch.getTime() << "ms\n";
  }
  return true;
}

bool sure::SUREFeatureExtractor::extractFeatures()
{
  Vector3 normalOrientationPoint(input_->sensor_origin_[0], input_->sensor_origin_[1], input_->sensor_origin_[2]);
  Scalar samplingrate = config.DescriptorSamplingrate;
  unsigned numberOfDistanceClasses(config.DescriptorNumberOfDistanceClasses);

  pcl::StopWatch watch;

  unsigned descriptors = sure::feature::createDescriptors(octree, features, samplingrate, normalOrientationPoint, numberOfDistanceClasses);

  if( verbose )
  {
    std::cout << "Calculated " << descriptors << " descriptors in " << watch.getTime() << "ms\n";
  }

  return true;
}

