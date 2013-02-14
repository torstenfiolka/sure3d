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

float sure::calculateCornerness(const std::vector<OctreeNode*>& nodes)
{
  Eigen::Vector3f mean = Eigen::Vector3f::Zero(), direction;
  float sumWeight = 0.f;
  int count = 0;
  Eigen::Matrix3f covMatrix = Eigen::Matrix3f::Zero();

  for(unsigned int j=0; j<nodes.size(); ++j)
  {
    if( nodes[j]->value.entropy != 0.f )
    {
      mean[0] += nodes[j]->value.entropy * nodes[j]->closestPosition.p[0];
      mean[1] += nodes[j]->value.entropy * nodes[j]->closestPosition.p[1];
      mean[2] += nodes[j]->value.entropy * nodes[j]->closestPosition.p[2];

      sumWeight += nodes[j]->value.entropy;
      count++;
    }
  }
  if( sumWeight > 0.f )
  {
    mean /= sumWeight;
  }
  else
  {
    return INFINITY;
  }
  for(unsigned int j=0; j<nodes.size(); ++j)
  {
    if( nodes[j]->value.entropy != 0.f )
    {
      direction[0] = (mean[0] - nodes[j]->closestPosition.p[0]);
      direction[1] = (mean[1] - nodes[j]->closestPosition.p[1]);
      direction[2] = (mean[2] - nodes[j]->closestPosition.p[2]);

      covMatrix += nodes[j]->value.entropy * (direction * direction.transpose());
    }
  }
  covMatrix /= sumWeight;

  Eigen::Matrix<float, 3, 1> eigenValues;
  Eigen::Matrix<float, 3, 3> eigenVectors;
  pcl::eigen33(covMatrix, eigenVectors, eigenValues);
  return eigenValues(0) / eigenValues(2);
}

//
//  Methods for data and organisational purposes
//

//! resets the current object
template <typename PointT>
void sure::SURE_Estimator<PointT>::reset()
{
  if( octree )
    delete octree;
  octree = NULL;
  octreeDepth = 0;
  octreeNodeSizeByDepth.clear();
  config = sure::Configuration();
  features.clear();
  rangeImage.reset();
  octreeMap.clear();
  addedPoints.clear();
//  viewPoint = Eigen::Vector3f::Zero();
}

/**
 * Sets a new maximum octree size
 * @param size
 */
template <typename PointT>
void sure::SURE_Estimator<PointT>::resize(unsigned int size)
{
  reset();
  if( size != currentOctreeSize && size > 0 )
  {
    currentOctreeSize = size;
    if( octreeAllocator )
    {
      delete octreeAllocator;
    }
    octreeAllocator = new OctreeAllocator(currentOctreeSize);
  }
}

//
//  Getters and Setters
//

template <typename PointT>
sure::OctreeNode* sure::SURE_Estimator<PointT>::getNodeFromPosition(const PointT& point, int level)
{
  OctreePosition pos;
  pos.p[0] = point.x;
  pos.p[1] = point.y;
  pos.p[2] = point.z;
  return getNodeFromPosition(pos, level);
}

template <typename PointT>
sure::OctreeNode* sure::SURE_Estimator<PointT>::getNodeFromPosition(const Eigen::Vector3f& posVec, int level)
{
  sure::OctreePosition pos;
  pos.p[0] = posVec[0];
  pos.p[1] = posVec[1];
  pos.p[2] = posVec[2];
  return getNodeFromPosition(pos, level);
}

/**
 * Returns the octree node containing a given position on a given level, or the closest node, if the position lies not within the octree
 * @param pos
 * @param level
 * @return
 */
template <typename PointT>
sure::OctreeNode* sure::SURE_Estimator<PointT>::getNodeFromPosition(const sure::OctreePosition& pos, int level)
{
  sure::OctreeNode* closest = NULL;
  float distance, bestDistance = INFINITY;
  if( level == 0 )
  {
    level = config.samplingLevel;
  }
  for(unsigned int i=0; i<octreeMap[level].size(); ++i)
  {
    if( octreeMap[level][i]->inRegion(pos) )
    {
      return octreeMap[level][i];
    }
  }
  for(unsigned int i=0; i<octreeMap[level].size(); ++i)
  {
    distance = (pos.p[0] - octreeMap[level][i]->closestPosition.p[0])*(pos.p[0] - octreeMap[level][i]->closestPosition.p[0]) + (pos.p[1] - octreeMap[level][i]->closestPosition.p[1])*(pos.p[1] - octreeMap[level][i]->closestPosition.p[1]) + (pos.p[2] - octreeMap[level][i]->closestPosition.p[2])*(pos.p[2] - octreeMap[level][i]->closestPosition.p[2]);
    if( distance < bestDistance )
    {
      bestDistance = distance;
      closest = octreeMap[level][i];
    }
  }
  return closest;
}

template <typename PointT>
pcl::PointCloud<pcl::InterestPoint>::Ptr sure::SURE_Estimator<PointT>::getInterestPoints() const
{
  pcl::PointCloud<pcl::InterestPoint>::Ptr interestPoints(new pcl::PointCloud<pcl::InterestPoint>);
  interestPoints->reserve(features.size());
  for(std::vector<sure::Feature>::const_iterator it=features.begin(); it != features.end(); ++it)
  {
    pcl::InterestPoint p;
    p.x = (*it).position()[0];
    p.y = (*it).position()[1];
    p.z = (*it).position()[2];
    p.strength = (*it).entropy();
    interestPoints->points.push_back(p);
  }
  interestPoints->header = input_->header;
  return interestPoints;
}

//
//  Methodes for the Octree
//

/**
 * Inserts a point into the octree with given status (used for inserting artificial points, since they must not be used for description)
 * @param p
 * @param status
 */
template <typename PointT>
void sure::SURE_Estimator<PointT>::insertPointInOctree(const PointT& p, sure::OctreeValue::NodeStatus status)
{
  sure::OctreeNode* n = NULL;

  sure::OctreePoint point;
  point.value.summedPos[0] = point.position.p[0] = p.x;
  point.value.summedPos[1] = point.position.p[1] = p.y;
  point.value.summedPos[2] = point.position.p[2] = p.z;

  point.value.summedSquares[0] = point.position.p[0] * point.position.p[0]; // xx
  point.value.summedSquares[1] = point.value.summedSquares[3] = point.position.p[0] * point.position.p[1]; // xy
  point.value.summedSquares[2] = point.value.summedSquares[6] = point.position.p[0] * point.position.p[2]; // xz
  point.value.summedSquares[4] = point.position.p[1] * point.position.p[1]; // yy
  point.value.summedSquares[5] = point.value.summedSquares[7] = point.position.p[1] * point.position.p[2]; // yz
  point.value.summedSquares[8] = point.position.p[2] * point.position.p[2]; // zz

  point.value.numberOfPoints = 1;
  point.value.statusOfMaximum = status;

  sure::convertPCLRGBtoFloatRGB(p.rgb, point.value.colorR, point.value.colorG, point.value.colorB);

  if( config.limitOctreeResolution )
  {
    float volumeSize, distance;
    distance = (input_->sensor_origin_(0) - point.position.p[0]) * (input_->sensor_origin_(0) - point.position.p[0])
             + (input_->sensor_origin_(1) - point.position.p[1]) * (input_->sensor_origin_(1) - point.position.p[1])
             + (input_->sensor_origin_(2) - point.position.p[2]) * (input_->sensor_origin_(2) - point.position.p[2]);
    volumeSize = std::max(config.octreeMinimumVolumeSize, config.getOctreeResolutionThreshold() * distance);
    n = octree->root->addPoint(point, volumeSize);
  }
  else
  {
    n = octree->addPoint(point);
  }

  if( n )
  {
    octreeDepth = std::max(octreeDepth, n->depth);
  }
}

//! (re)builds the octree and the octree sampling map
template <typename PointT>
void sure::SURE_Estimator<PointT>::buildOctree()
{
  if( octree )
    delete octree;
  if( octreeAllocator )
    octreeAllocator->reset();
  OctreePosition origin, maximum;
  origin.p[0] = origin.p[1] = origin.p[2] = 0.f;
  maximum.p[0] = maximum.p[1] = maximum.p[2] = config.getOctreeExpansion();
  octree = new Octree(maximum, origin, config.getOctreeMinimumVolumeSize(), octreeAllocator);

  octreeDepth = 0;

  if( config.ignoreBackgroundDetections || config.additionalPointsOnDepthBorders )
  {
    rangeImage.setInputCloud(input_);
    rangeImage.setIndices(indices_);
    rangeImage.calculateRangeImage();
  }

  if( config.ignoreBackgroundDetections )
  {
    for(unsigned int i=0; i<indices_->size(); ++i)
    {
      const PointT& point = input_->points[indices_->at(i)];
      if( std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) )
      {
        rangeImage.isBackgroundBorder(indices_->at(i)) ? insertPointInOctree(point, sure::OctreeValue::BACKGROUND) : insertPointInOctree(point);
      }
    }
  }
  else
  {
    for(unsigned int i=0; i<indices_->size(); ++i)
    {
      const PointT& point = input_->points[indices_->at(i)];
      if( std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) )
      {
        insertPointInOctree(point);
      }
    }
  }

  if( config.additionalPointsOnDepthBorders )
  {
    continueDepthBorders();
  }

  octreeNodeSizeByDepth.clear();
  octreeNodeSizeByDepth.resize(octreeDepth+1);

  resampleOctreeSamplingMap();
}

//! (re)build the Octree Sampling Map
template <typename PointT>
inline void sure::SURE_Estimator<PointT>::resampleOctreeSamplingMap()
{
  octreeMap.clear();
  octreeMap = algorithm::downsampleOcTree(*octree, false, octreeDepth);

  octreeSize = 0;
  float avgVolumeSize = 0.f;
  for(unsigned int level=0; level<=octreeDepth; ++level)
  {
    avgVolumeSize = 0.f;
    for(unsigned int j=0; j<octreeMap[level].size(); ++j)
    {
      avgVolumeSize += octreeMap[level][j]->maxPosition.p[0] - octreeMap[level][j]->minPosition.p[0];
    }
    octreeNodeSizeByDepth[level] = avgVolumeSize / (float) octreeMap[level].size();
    octreeSize += octreeMap[level].size();
  }
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::continueDepthBorders()
{
  rangeImage.addPointsOnBorders(config.octreeMinimumVolumeSize, config.histogramSize, addedPoints);
  for(unsigned int i=0; i<addedPoints.size(); ++i)
  {
    insertPointInOctree(addedPoints[i], sure::OctreeValue::ARTIFICIAL);
  }
}


//
// Normal calculation
//

template <typename PointT>
void sure::SURE_Estimator<PointT>::calculateNormals()
{
  calculateNormals(config.normalSamplingLevel, config.normalScaleRadius);
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::calculateNormals(unsigned int level, float radius)
{
//  std::cout << "Calculating normals on level " << level << " (" << octreeNodeSizeByDepth[level] << " cm) with radius " << radius << " cm." << std::endl;

//#pragma omp parallel for schedule(dynamic)
  for(std::vector<OctreeNode*>::iterator it=octreeMap[level].begin(); it!=octreeMap[level].end(); ++it)
  {
    calculateNormal((*it), radius);
    orientateNormal((*it)->value.normal[0], (*it)->value.normal[1], (*it)->value.normal[2], (*it)->closestPosition);
    (*it)->value.calculateHistogram();
  }
}

/**
 * Calculates a normal
 * @param value the value class with information for the covariance matrix
 * @param count the number of the points in value object
 * @param normal reference for the calculated normal
 * @param histogram reference for the histogram into which the normal is stored
 * @param pos position of the normal for view vector calculation
 * @return true, if the calculation is succesful
 */
template <typename PointT>
bool sure::SURE_Estimator<PointT>::calculateNormal(sure::OctreeNode* node, float radius, int count)
{
  if( count < 0 )
  {
    count = node->numPoints;
  }

  if( count < sure::MINIMUM_POINTS_FOR_NORMAL )
  {
    node->value.statusOfNormal = sure::OctreeValue::NORMAL_UNSTABLE;
    return false;
  }

  Eigen::Matrix3f summedSquares;
  Eigen::Vector3f summedPosition;


  if( radius > node->maxPosition.p[0] - node->position.p[0] )
  {
    OctreePosition minPosition, maxPosition;
    sure::OctreeValue tempValue;

    minPosition.p[0] = node->closestPosition.p[0] - radius;
    minPosition.p[1] = node->closestPosition.p[1] - radius;
    minPosition.p[2] = node->closestPosition.p[2] - radius;

    maxPosition.p[0] = node->closestPosition.p[0] + radius;
    maxPosition.p[1] = node->closestPosition.p[1] + radius;
    maxPosition.p[2] = node->closestPosition.p[2] + radius;

    unsigned int count = 0;
    octree->getValueAndCountInVolume(tempValue, count, minPosition, maxPosition, config.getOctreeMinimumVolumeSize());

    summedSquares(0, 0) = tempValue.summedSquares[0] * (1.f / (float) count);
    summedSquares(0, 1) = tempValue.summedSquares[1] * (1.f / (float) count);
    summedSquares(0, 2) = tempValue.summedSquares[2] * (1.f / (float) count);
    summedSquares(1, 0) = tempValue.summedSquares[3] * (1.f / (float) count);
    summedSquares(1, 1) = tempValue.summedSquares[4] * (1.f / (float) count);
    summedSquares(1, 2) = tempValue.summedSquares[5] * (1.f / (float) count);
    summedSquares(2, 0) = tempValue.summedSquares[6] * (1.f / (float) count);
    summedSquares(2, 1) = tempValue.summedSquares[7] * (1.f / (float) count);
    summedSquares(2, 2) = tempValue.summedSquares[8] * (1.f / (float) count);

    summedPosition(0) = tempValue.summedPos[0] * (1.f / (float) count);
    summedPosition(1) = tempValue.summedPos[1] * (1.f / (float) count);
    summedPosition(2) = tempValue.summedPos[2] * (1.f / (float) count);
  }
  else
  {
    summedSquares(0, 0) = node->value.summedSquares[0] * (1.f / (float) count);
    summedSquares(0, 1) = node->value.summedSquares[1] * (1.f / (float) count);
    summedSquares(0, 2) = node->value.summedSquares[2] * (1.f / (float) count);
    summedSquares(1, 0) = node->value.summedSquares[3] * (1.f / (float) count);
    summedSquares(1, 1) = node->value.summedSquares[4] * (1.f / (float) count);
    summedSquares(1, 2) = node->value.summedSquares[5] * (1.f / (float) count);
    summedSquares(2, 0) = node->value.summedSquares[6] * (1.f / (float) count);
    summedSquares(2, 1) = node->value.summedSquares[7] * (1.f / (float) count);
    summedSquares(2, 2) = node->value.summedSquares[8] * (1.f / (float) count);

    summedPosition(0) = node->value.summedPos[0] * (1.f / (float) count);
    summedPosition(1) = node->value.summedPos[1] * (1.f / (float) count);
    summedPosition(2) = node->value.summedPos[2] * (1.f / (float) count);
  }

  summedSquares -= summedPosition * summedPosition.transpose();

  if( !sure::is_finite(summedSquares) )
  {
    return false;
  }

  Eigen::Matrix3f eigenVectors;
  Eigen::Vector3f eigenValues;
  pcl::eigen33(summedSquares, eigenVectors, eigenValues);

  if( std::isfinite(eigenVectors.col(0)[0]) && std::isfinite(eigenVectors.col(0)[1]) && std::isfinite(eigenVectors.col(0)[2]) )
  {
    node->value.statusOfNormal = sure::OctreeValue::NORMAL_STABLE;
    node->value.eigenValues[0] = eigenValues[0];
    node->value.eigenValues[1] = eigenValues[1];
    node->value.eigenValues[2] = eigenValues[2];
    node->value.normal[0] = node->value.eigenVectors[0] = eigenVectors.col(0)[0];
    node->value.normal[1] = node->value.eigenVectors[1] = eigenVectors.col(0)[1];
    node->value.normal[2] = node->value.eigenVectors[2] = eigenVectors.col(0)[2];
    node->value.eigenVectors[3] = eigenVectors.col(1)[0];
    node->value.eigenVectors[4] = eigenVectors.col(1)[1];
    node->value.eigenVectors[5] = eigenVectors.col(1)[2];
    node->value.eigenVectors[6] = eigenVectors.col(2)[0];
    node->value.eigenVectors[7] = eigenVectors.col(2)[1];
    node->value.eigenVectors[8] = eigenVectors.col(2)[2];
    node->value.test[0] = eigenValues[0] / (eigenValues[0]+eigenValues[1]+eigenValues[2]);
    return true;
  }
  else
  {
    node->value.normal[0] = node->value.normal[1] = node->value.normal[2] = 0.f;
    node->value.statusOfNormal = sure::OctreeValue::NORMAL_UNSTABLE;
  }
  return false;
}

template <typename PointT>
bool sure::SURE_Estimator<PointT>::calculateNormal(const Eigen::Vector3f& position, float radius, Eigen::Vector3f& normal)
{
  sure::OctreeNode node;

  node.minPosition.p[0] = position[0] - radius;
  node.minPosition.p[1] = position[1] - radius;
  node.minPosition.p[2] = position[2] - radius;

  node.maxPosition.p[0] = position[0] + radius;
  node.maxPosition.p[1] = position[1] + radius;
  node.maxPosition.p[2] = position[2] + radius;

  node.value.statusOfMaximum = sure::OctreeValue::ARTIFICIAL;
  octree->getValueAndCountInVolume(node.value, node.numPoints, node.minPosition, node.maxPosition, config.getOctreeMinimumVolumeSize());

  if( calculateNormal(&node) )
  {
    normal = Eigen::Vector3f(node.value.normal[0], node.value.normal[1], node.value.normal[2]);
    return true;
  }
  else
  {
    normal = Eigen::Vector3f::Zero();
  }
  return false;
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::orientateNormal(Eigen::Vector3f& normal, const Eigen::Vector3f& position)
{
  Eigen::Vector3f viewVector(input_->sensor_origin_[0]-position[0], input_->sensor_origin_[1]-position[1], input_->sensor_origin_[2]-position[2]);
  if( normal.dot(viewVector) < 0.f )
  {
    normal = -normal;
  }
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::orientateNormal(Eigen::Vector3f& normal, const OctreePosition& position)
{
  Eigen::Vector3f viewVector(input_->sensor_origin_[0]-position.p[0], input_->sensor_origin_[1]-position.p[1], input_->sensor_origin_[2]-position.p[2]);
  if( normal.dot(viewVector) < 0.f )
  {
    normal = -normal;
  }
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::orientateNormal(float& normal1, float& normal2, float& normal3, const OctreePosition& position)
{
  Eigen::Vector3f normal(normal1, normal2, normal3);
  Eigen::Vector3f viewVector(input_->sensor_origin_[0]-position.p[0], input_->sensor_origin_[1]-position.p[1], input_->sensor_origin_[2]-position.p[2]);
  if( normal.dot(viewVector) < 0.f )
  {
    normal1 = -normal1;
    normal2 = -normal2;
    normal3 = -normal3;
  }
}

//
//  Feature calculation
//

/**
 * Main method for creating features. It calculates normal-histograms and entropy, searchs for local maxima and
 * extracts features
 */
template <typename PointT>
void sure::SURE_Estimator<PointT>::calculateFeatures()
{
  initCompute();
//  std::cout << "Pointcloud: " << input_->size() << " Indices: " << indices_->size() << std::endl;
  features.clear();

//  std::cout << "Octree";
  buildOctree();

//  std::cout << "(" << octreeSize << ") Normals";
  calculateNormals();

//  std::cout << " Entropy";
  calculateEntropy();

//  std::cout << " Extraction";
  extractFeature();

  if( config.improvedLocalization )
  {
//    std::cout << " Localization";
    localizeFeatureWithMeanShift(3);
  }
  std::cout << std::endl << "Calculated " << this->features.size() << " features.\n";
}

//! Calculates entropy for all nodes on the configured depth
template <typename PointT>
void sure::SURE_Estimator<PointT>::calculateEntropy()
{
  calculateEntropy(config.samplingLevel, config.histogramRadius);
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::calculateEntropy(unsigned int level, float radius)
{
  for(std::vector<OctreeNode*>::iterator it=octreeMap[level].begin(); it!=octreeMap[level].end(); ++it)
  {
    (*it)->value.entropy = calculateEntropy((*it), radius);
  }
}

template <typename PointT>
void sure::SURE_Estimator<PointT>::calculateEntropy(unsigned int level, float radius, unsigned int index)
{
  for(std::vector<OctreeNode*>::iterator it=octreeMap[level].begin(); it!=octreeMap[level].end(); ++it)
  {
    (*it)->value.test[index] = calculateEntropy((*it), radius);
  }
}

//! Calculates the entropy on a single node using a histogram of normals
template <typename PointT>
float sure::SURE_Estimator<PointT>::calculateEntropy(OctreeNode* treenode, float radius)
{
  std::vector<OctreeNode*> listOfNodes;
  OctreePosition minPosition, maxPosition;

  sure::NormalHistogram entropyHistogram;

  minPosition.p[0] = treenode->closestPosition.p[0] - radius;
  minPosition.p[1] = treenode->closestPosition.p[1] - radius;
  minPosition.p[2] = treenode->closestPosition.p[2] - radius;
  maxPosition.p[0] = treenode->closestPosition.p[0] + radius;
  maxPosition.p[1] = treenode->closestPosition.p[1] + radius;
  maxPosition.p[2] = treenode->closestPosition.p[2] + radius;

  octree->root->getAllNodesInVolumeOnSamplingDepth(listOfNodes, minPosition, maxPosition, config.normalSamplingLevel, false);

  Eigen::Vector3f refNormal;
  if( config.entropyMode == sure::CROSSPRODUCTS_ALL_NORMALS_WITH_MAIN_NORMAL )
  {
    Eigen::Vector3f pos(treenode->closestPosition.p[0], treenode->closestPosition.p[1], treenode->closestPosition.p[2]);
    if( !calculateNormal(pos, radius, refNormal) )
    {
      return 0.f;
    }
  }

  switch( config.entropyMode )
  {
    default:
    case sure::NORMALS:
      for(std::vector<OctreeNode*>::iterator it=listOfNodes.begin(); it!=listOfNodes.end(); ++it)
      {
        if( (*it)->value.statusOfNormal == sure::OctreeValue::NORMAL_STABLE )
        {
          entropyHistogram += (*it)->value.getHistogram();
        }
      }
      break;
    case sure::CROSSPRODUCTS_ALL_NORMALS_WITH_MAIN_NORMAL:
      for(std::vector<OctreeNode*>::iterator it=listOfNodes.begin(); it!=listOfNodes.end(); ++it)
      {
        if( (*it)->value.statusOfNormal == sure::OctreeValue::NORMAL_STABLE )
        {
          Eigen::Vector3f secNormal((*it)->value.normal[0], (*it)->value.normal[1], (*it)->value.normal[2]);
          entropyHistogram.insertCrossProduct(refNormal, secNormal, (sure::CrossProductWeightMethod) config.cpWeightMethod);
        }
      }
      break;
    case sure::CROSSPRODUCTS_ALL_NORMALS_PAIRWISE:
      for(std::vector<OctreeNode*>::iterator first=listOfNodes.begin(); first!=listOfNodes.end(); ++first)
      {
        for(std::vector<OctreeNode*>::iterator second=first+1; second!=listOfNodes.end(); ++second)
        {
          if( (*first)->value.statusOfNormal == sure::OctreeValue::NORMAL_STABLE && (*second)->value.statusOfNormal == sure::OctreeValue::NORMAL_STABLE )
          {
            refNormal = Eigen::Vector3f((*first)->value.normal[0], (*first)->value.normal[1], (*first)->value.normal[2]);
            Eigen::Vector3f secNormal((*second)->value.normal[0], (*second)->value.normal[1], (*second)->value.normal[2]);
            entropyHistogram.insertCrossProduct(refNormal, secNormal, (sure::CrossProductWeightMethod) config.cpWeightMethod);
          }
        }
      }
      break;
  }

//  entropyHistogram.print();
  entropyHistogram.calculateEntropy();
  return entropyHistogram.entropy;
}

//! Calculates the internal feature map
//! searchs and marks local maxima
template <typename PointT>
void sure::SURE_Estimator<PointT>::extractFeature()
{
  OctreePosition minPosition, maxPosition;

  unsigned int& level = config.samplingLevel;

  std::vector< OctreeNode* > neighbours;
  neighbours.reserve(floor((config.histogramSize/config.samplingRate)*(config.histogramSize/config.samplingRate)*(config.histogramSize/config.samplingRate)));

  for(unsigned int i=0; i<octreeMap[level].size(); ++i)
  {
    OctreeNode* currentNode = octreeMap[level][i];
    if( currentNode->value.statusOfMaximum == sure::OctreeValue::ARTIFICIAL || currentNode->value.statusOfMaximum == sure::OctreeValue::BACKGROUND )
    {
      continue;
    }
    // Check for threshold
    if( currentNode->value.entropy < config.minimumEntropy )
    {
      currentNode->value.statusOfMaximum = sure::OctreeValue::MAXIMUM_NOT_POSSIBLE;
      continue;
    }

    if( currentNode->value.statusOfMaximum == sure::OctreeValue::MAXIMUM_NOT_CALCULATED && config.minimumCornerness3D > 0.f )
    {
      OctreeNode* currentNode = octreeMap[level][i];
      neighbours.clear();

      // Collect Neighbors
      minPosition.p[0] = currentNode->closestPosition.p[0] - config.histogramRadius;
      minPosition.p[1] = currentNode->closestPosition.p[1] - config.histogramRadius;
      minPosition.p[2] = currentNode->closestPosition.p[2] - config.histogramRadius;
      maxPosition.p[0] = currentNode->closestPosition.p[0] + config.histogramRadius;
      maxPosition.p[1] = currentNode->closestPosition.p[1] + config.histogramRadius;
      maxPosition.p[2] = currentNode->closestPosition.p[2] + config.histogramRadius;

      octree->root->getAllNodesInVolumeOnSamplingDepth(neighbours, minPosition, maxPosition, level, false);
      currentNode->value.cornerness3D = sure::calculateCornerness(neighbours);
      if( currentNode->value.cornerness3D < config.minimumCornerness3D )
      {
        currentNode->value.statusOfMaximum = sure::OctreeValue::MAXIMUM_NOT_POSSIBLE;
      }
      else
      {
        currentNode->value.statusOfMaximum = sure::OctreeValue::MAXIMUM_POSSIBLE;
      }
    }
    else if( config.minimumCornerness3D == 0.f )
    {
      currentNode->value.statusOfMaximum  = sure::OctreeValue::MAXIMUM_POSSIBLE;
    }
  }

  for(unsigned int i=0; i<octreeMap[level].size(); ++i)
  {
    OctreeNode* currentNode = octreeMap[level][i];
    if( currentNode->value.statusOfMaximum != sure::OctreeValue::MAXIMUM_POSSIBLE )
    {
      continue;
    }

    neighbours.clear();

    minPosition.p[0] = currentNode->closestPosition.p[0] - config.featureInfluenceRadius;
    minPosition.p[1] = currentNode->closestPosition.p[1] - config.featureInfluenceRadius;
    minPosition.p[2] = currentNode->closestPosition.p[2] - config.featureInfluenceRadius;
    maxPosition.p[0] = currentNode->closestPosition.p[0] + config.featureInfluenceRadius;
    maxPosition.p[1] = currentNode->closestPosition.p[1] + config.featureInfluenceRadius;
    maxPosition.p[2] = currentNode->closestPosition.p[2] + config.featureInfluenceRadius;

    octree->root->getAllNodesInVolumeOnSamplingDepth(neighbours, minPosition, maxPosition, level, false);

    const float& referenceEntropy = currentNode->value.entropy;
    bool isMaximum = true;

    for(unsigned int j=0; j<neighbours.size(); ++j)
    {
      // Check neighbors for maximum
      if( neighbours[j] == currentNode )
      {
        continue;
      }
      if( neighbours[j]->value.statusOfMaximum == sure::OctreeValue::MAXIMUM_FOUND )
      {
        isMaximum = false;
        break;
      }
      if( neighbours[j]->value.statusOfMaximum == sure::OctreeValue::MAXIMUM_POSSIBLE && neighbours[j]->value.entropy > referenceEntropy )
      {
        isMaximum = false;
        break;
      }
    }

    // If feature is still marked, push to featureVector
    if( isMaximum )
    {
      octreeMap[level][i]->value.statusOfMaximum = sure::OctreeValue::MAXIMUM_FOUND;
      features.push_back(createFeature(octreeMap[level][i]));
    }
  }
}

//! improves localization with mean shift
//! Localizes the Features with Mean-Shift
template <typename PointT>
void sure::SURE_Estimator<PointT>::localizeFeatureWithMeanShift(int iterations)
{
//  ROS_DEBUG_NAMED("octree", "Starting feature localization improvement with mean shift in %i iterations...", iterations);

  unsigned int level = config.samplingLevel;
  float searchRadius = config.histogramRadius;

  std::vector<OctreeNode*> listOfNodes;
  listOfNodes.reserve(floor((config.histogramSize/config.samplingRate)*(config.histogramSize/config.samplingRate)*(config.histogramSize/config.samplingRate)));
  sure::Feature newFeature;
  OctreePosition minPosition, maxPosition, shiftedPosition;
  int count;
  float summedShift, mean, standardDeviation, summedMean, summedSquares, entropyDifference;


  for(unsigned int featureIndex=0; featureIndex<features.size(); ++featureIndex)
  {
    if( features[featureIndex].radius() != config.histogramRadius )
    {
      continue;
    }

    listOfNodes.clear();
    newFeature = features[featureIndex];

    for(int iteration=0; iteration<iterations; ++iteration)
    {
      summedShift = 0.f, mean = 0.f, standardDeviation = 0.f, summedMean = 0.f, summedSquares = 0.f, entropyDifference = 0.f;

      shiftedPosition.p[0] = shiftedPosition.p[1] = shiftedPosition.p[2] = 0.f;
      minPosition.p[0] = newFeature.position()[0] - searchRadius;
      minPosition.p[1] = newFeature.position()[1] - searchRadius;
      minPosition.p[2] = newFeature.position()[2] - searchRadius;
      maxPosition.p[0] = newFeature.position()[0] + searchRadius;
      maxPosition.p[1] = newFeature.position()[1] + searchRadius;
      maxPosition.p[2] = newFeature.position()[2] + searchRadius;

      listOfNodes.clear();
      octree->root->getAllNodesInVolumeOnSamplingDepth(listOfNodes, minPosition, maxPosition, level, false);

      count = 0;

      for(unsigned int i=0; i<listOfNodes.size(); ++i)
      {
        if( listOfNodes[i]->value.statusOfMaximum == sure::OctreeValue::ARTIFICIAL )
        {
          continue;
        }
//        Eigen::Vector3f pos(listOfNodes[i]->closestPosition.p[0], listOfNodes[i]->closestPosition.p[1], listOfNodes[i]->closestPosition.p[2]);
//        if( (pos-featureVector[featureIndex].pos.point).norm() > config.histogramRadius )
//        {
//          continue;
//        }
        summedMean += listOfNodes[i]->value.entropy;
        summedSquares += listOfNodes[i]->value.entropy * listOfNodes[i]->value.entropy;
        count++;
      }

      if( count > 0 )
      {
        mean = summedMean / float(count);
        summedMean = summedMean*summedMean;
        standardDeviation = sqrt(fabs(summedSquares - ((1.f/float(count)) * summedMean)));
        for(unsigned int i=0; i<listOfNodes.size(); ++i)
        {
          entropyDifference = mean - listOfNodes[i]->value.entropy;
          shiftedPosition += listOfNodes[i]->closestPosition * exp(-0.5f*(entropyDifference*entropyDifference) / (standardDeviation*standardDeviation));
          summedShift += exp(-0.5f*(entropyDifference*entropyDifference) / (standardDeviation*standardDeviation));
        }
        if( summedShift != 0 )
        {
          shiftedPosition = shiftedPosition * (1.f / summedShift);
        }
        if( !isnan(shiftedPosition.p[0]) && !isnan(shiftedPosition.p[1]) && !isnan(shiftedPosition.p[2]) )
        {
  //        float distance = sqrt((shiftedPosition.p[0] - newFeature.pos.point[0])*(shiftedPosition.p[0] - newFeature.pos.point[0]) + (shiftedPosition.p[1] - newFeature.pos.point[1])*(shiftedPosition.p[1] - newFeature.pos.point[1]) + (shiftedPosition.p[2] - newFeature.pos.point[2])*(shiftedPosition.p[2] - newFeature.pos.point[2]));
  //        ROS_DEBUG_NAMED("octree", "Mean Shift - Iteration: %i\t Distance: %f", iteration, distance);
          newFeature.setPosition(shiftedPosition.p[0], shiftedPosition.p[1], shiftedPosition.p[2]);
        }
      }
    }
    createDescriptor(newFeature);
    features[featureIndex] = newFeature;
  }
//  ROS_DEBUG_NAMED("octree", "Mean shift localization finished in %f seconds.", ros::Duration(ros::Time::now() - start).toSec());
}

//! Creates a feature given a node
//! Extracts a feature from a given octree node
template <typename PointT>
sure::Feature sure::SURE_Estimator<PointT>::createFeature(OctreeNode* node)
{
  sure::Feature newFeature;
  newFeature.setPointCloudIndex(node->value.pointCloudIndex);
  newFeature.setEntropy(node->value.entropy);
  newFeature.setPosition(node->closestPosition.p[0], node->closestPosition.p[1], node->closestPosition.p[2]);
  newFeature.setRadius(config.histogramRadius);
  createDescriptor(newFeature);
  return newFeature;
}

/**
 * Extracts a feature on a given position with the current settings, but does not calculate the entropy
 * @param point in xyz-coordinates
 * @return
 */
template <typename PointT>
sure::Feature sure::SURE_Estimator<PointT>::createFeature(const Eigen::Vector3f& point)
{
  OctreePosition minPosition, maxPosition, closestPosition;
  std::vector<OctreeNode* > neighbours;
  sure::OctreeValue value;
  sure::Feature nf;

  closestPosition.p[0] = point[0];
  closestPosition.p[1] = point[1];
  closestPosition.p[2] = point[2];
  minPosition.p[0] = point[0] - config.samplingRate / 2.f;
  minPosition.p[1] = point[1] - config.samplingRate / 2.f;
  minPosition.p[2] = point[2] - config.samplingRate / 2.f;
  maxPosition.p[0] = point[0] + config.samplingRate / 2.f;
  maxPosition.p[1] = point[1] + config.samplingRate / 2.f;
  maxPosition.p[2] = point[2] + config.samplingRate / 2.f;
  nf.setPosition(point);
  nf.setRadius(config.histogramRadius);
  createDescriptor(nf);
  return nf;
}

//! Creates a feature for a point in the input point cloud

template <typename PointT>
sure::Feature sure::SURE_Estimator<PointT>::createFeature(int index)
{
  Eigen::Vector3f pos;
  sure::Feature feature;
  if( index >= 0 && index < (int) input_->points.size() && std::isfinite(input_->points[index].x) && std::isfinite(input_->points[index].y) && std::isfinite(input_->points[index].z) )
  {
    pos[0] = input_->points[index].x;
    pos[1] = input_->points[index].y;
    pos[2] = input_->points[index].z;
    feature.setColor(input_->points[index].rgb);
    feature = createFeature(pos);
    feature.setPointCloudIndex(index);
  }
  return feature;
}

//! Creates the descriptor for a given interest point

template <typename PointT>
void sure::SURE_Estimator<PointT>::createDescriptor(sure::Feature& feature)
{
  OctreePosition minPosition, maxPosition;

  Eigen::Vector3f normal(Eigen::Vector3f::Zero());

  calculateNormal(feature.position(), config.histogramRadius, normal);
  orientateNormal(normal, feature.position());

  feature.setNormal(normal);

  minPosition.p[0] = feature.position()[0] - config.histogramRadius;
  minPosition.p[1] = feature.position()[1] - config.histogramRadius;
  minPosition.p[2] = feature.position()[2] - config.histogramRadius;
  maxPosition.p[0] = feature.position()[0] + config.histogramRadius;
  maxPosition.p[1] = feature.position()[1] + config.histogramRadius;
  maxPosition.p[2] = feature.position()[2] + config.histogramRadius;

  sure::OctreeValue value = octree->getValueInVolume(minPosition, maxPosition);
  feature.setColor(value.r(), value.g(), value.b());

  std::vector<OctreeNode*> nodes;

  octree->root->getAllNodesInVolumeOnSamplingDepth(nodes, minPosition, maxPosition, config.normalSamplingLevel, false);

  feature.calculateDescriptor(nodes);
  feature.setCornerness(sure::calculateCornerness(nodes));
}
