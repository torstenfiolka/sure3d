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

#include <iostream>

#include <sure/sure.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time.hpp>

#include <stdlib.h>

pcl::PointCloud<pcl::InterestPoint>::Ptr features;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRandomCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for(unsigned int i=0; i<1000000; ++i)
  {
    pcl::PointXYZRGB p;
    p.x = ((float) rand() / (float) RAND_MAX)*40.f - 20.f;
    p.y = ((float) rand() / (float) RAND_MAX)*40.f - 20.f;
    p.z = ((float) rand() / (float) RAND_MAX)*40.f - 20.f;
    cloud->points.push_back(p);
  }
  return cloud;
}

//! converts rgb-ints to the pcl-float for storing rgb-values
inline float createPCLRGBfromInt(int r, int g, int b)
{
  int32_t rgb = (r << 16) | (g << 8) | b;
  return *(float *)(&rgb);
}

//! converts rgb-floats to the pcl-float for storing rgb-values
inline float createPCLRGBFromFloat(float r, float g, float b)
{
  int ir, ig, ib;
  ir = floor(r*255.f+0.5f);
  ig = floor(g*255.f+0.5f);
  ib = floor(b*255.f+0.5f);
  return createPCLRGBfromInt(ir, ig, ib);
}

//! Generates a cube of 1 m edge length in the pointcloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->points.clear();
  cloud->points.reserve(300000);

  float start = -0.5f;
  float stop = 0.5f;
  float stepSize = 0.0025f;

  for (float x=start; x<=stop; x+=stepSize) {
    for (float y=start; y<=stop; y+=stepSize) {
      pcl::PointXYZRGB point;
      point.x = x + 0.0001f;
      point.y = y + 0.0001f;
      point.z = start + 0.0001f;
      point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));

      cloud->points.push_back(point);
      point.x = x + 0.0001f;
      point.y = y + 0.0001f;
      point.z = stop + 0.0001f;
      point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));

      cloud->points.push_back(point);
    }
  }
  for (float y=start; y<=stop; y+=stepSize) {
    for (float z=start; z<=stop; z+=stepSize) {
      pcl::PointXYZRGB point;
      point.x = start + 0.0001f;
      point.y = y + 0.0001f;
      point.z = z + 0.0001f;
      point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);

      point.x = stop + 0.0001f;
      point.y = y + 0.0001f;
      point.z = z + 0.0001f;
      point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);
    }
  }
  for (float x=start; x<=stop; x+=stepSize) {
    for (float z=start; z<=stop; z+=stepSize) {
      pcl::PointXYZRGB point;
      point.x = x + 0.0001f;
      point.y = start + 0.0001f;
      point.z = z + 0.0001f;
      point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);

      point.x = x + 0.0001f;
      point.y = stop + 0.0001f;
      point.z = z + 0.0001f;
      point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  cloud->header.frame_id = "/openni_rgb_optical_frame";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Rotate the cube
  Eigen::Affine3f transform(Eigen::Matrix3f(Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) ));
  pcl::transformPointCloud(*cloud, *transformedCloud, transform);

  transformedCloud->sensor_orientation_ = Eigen::Quaternionf(transform.rotation());
  transformedCloud->sensor_origin_ = Eigen::Vector4f(transform.translation()[0], transform.translation()[1], transform.translation()[2], 0.f);

  return transformedCloud;
}

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  if( features )
  {
    for(unsigned i=0; i<features->size(); ++i)
    {
      std::stringstream s;
      s << "feature" << i;
      pcl::PointXYZ p;
      p.x = features->at(i).x;
      p.y = features->at(i).y;
      p.z = features->at(i).z;
      viewer.addSphere(p, features->at(i).strength, s.str(), 0);
    }
  }
}

int main ()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = generatePointCloud();

    pcl::visualization::CloudViewer viewer("SURE 3D Features Visualization");

    // SURE 3D Feature base class
    sure::SUREFeatureExtractor sure;

    // set input cloud
    sure.setInputCloud(cloud);

    sure::Configuration& config = sure.config;

    // Adjust the size of the features (in meter)
    config.clearScales();
    config.addScale(0.12f);
//    config.addScale(0.24f);
//    config.addScale(0.36f);

    // Adjust the sampling rate
    config.Samplingrate = 0.04;

    // Adjust the normal sampling rate
    config.NormalSamplingrate = 0.02;

    // Adjust the influence radius for calculating normals
    config.NormalRegionSize = 0.04;

    // Adjust the minimum entropy for a feature
    config.MinimumEntropyThreshold = 0.6;

    // Adjust the minimum Cornerness to reduce number of features on edges
    config.MinimumCornernessThreshold = 0.3;

    config.AdditionalPointsOnDepthBorders = false;
    config.IgnoreBackgroundDetections = false;
    config.IgnoreNormalsOnBackgroundDepthBorders = false;

    sure.verbose = true;

    // calculate features
    sure.calculateSURE();

    // get a copy of the features for visualization
    features = sure.getInterestPoints();

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    while (!viewer.wasStopped ())
    {
    }
    return 0;
}
