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

#include <sure/keypoints/keypoint_calculation.h>

void sure::keypoints::allocateEntropyPayload(Octree& octree, Scalar samplingrate, sure::memory::FixedSizeAllocatorWithDirectAccess<EntropyPayload>& allocator)
{
  unsigned depth = octree.getDepth(samplingrate);
  allocator.resizeIfSmaller(octree[depth].size());
  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* node = octree[depth][i];
    EntropyPayload* payload = allocator.allocate(i);
    node->setOptionalPayload(static_cast<sure::payload::Payload*>(payload));
  }
}

void sure::keypoints::flagArtificialPoints(Octree& octree, Scalar samplingrate)
{
  unsigned depth = octree.getDepth(samplingrate);

  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* node = octree[depth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(node->opt());

    if( node->fixed().getPointFlag() == ARTIFICIAL )
    {
      payload->flag_ = ARTIFICIAL_POINTS;
      continue;
    }
  }
}

void sure::keypoints::flagDistantPoints(Octree& octree, Scalar samplingrate, Scalar threshold, const Vector3& sensorPosition)
{
  unsigned depth = octree.getDepth(samplingrate);
  threshold *= threshold;

  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* node = octree[depth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(node->opt());

    if( (node->fixed().getMeanPosition() - sensorPosition).squaredNorm() > threshold )
    {
      payload->flag_ = DISTANCE_TOO_HIGH;
      continue;
    }
  }
}

void sure::keypoints::flagBackgroundPoints(Octree& octree, Scalar samplingrate)
{
  unsigned depth = octree.getDepth(samplingrate);

  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* node = octree[depth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(node->opt());

    if( ((node->fixed().getPointFlag() & BACKGROUND_BORDER) == BACKGROUND_BORDER) )
    {
      payload->flag_ = BACKGROUND_EDGE;
      continue;
    }
  }
}

void sure::keypoints::resetFeatureFlags(Octree& octree, Scalar samplingrate)
{
  unsigned depth = octree.getDepth(samplingrate);

  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* node = octree[depth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(node->opt());

    if( payload->flag_ < DISTANCE_TOO_HIGH )
    {
      payload->flag_ = NOT_CALCULATED;
    }
  }
}


void sure::keypoints::calculateEntropy(Octree& octree, Scalar samplingrate, Scalar normalSamplingrate, Scalar radius, Scalar threshold, EntropyCalculationMode mode, Scalar influenceRadius)
{
  unsigned samplingDepth = octree.getDepth(samplingrate);

  for(unsigned int i=0; i<octree[samplingDepth].size(); ++i)
  {
    Node* node = octree[samplingDepth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(node->opt());

    if( payload->flag_ != NOT_CALCULATED )
    {
      continue;
    }

    switch( mode )
    {
      default:
      case NORMALS:
        payload->entropy_ = calculateEntropyWithNormals(octree, node, normalSamplingrate, radius, influenceRadius);
        break;
      case CROSS_PRODUCTS_W_MAIN:
        payload->entropy_ = calculateEntropyWithCrossproducts(octree, node, normalSamplingrate, radius, influenceRadius);
        break;
      case CROSS_PRODUCTS_PAIRWISE:
        payload->entropy_ = calculateEntropyWithCrossproductsPairwise(octree, node, normalSamplingrate, radius, influenceRadius);
        break;
    }

    if( payload->entropy_ < threshold )
    {
      payload->flag_ = ENTROPY_TOO_LOW;
      continue;
    }
    payload->flag_ = POSSIBLE;
  }
}

sure::Scalar sure::keypoints::calculateEntropyWithNormals(const Octree& octree, Node* node, Scalar normalSamplingrate, Scalar radius, Scalar influenceRadius)
{
  NormalPayload regionIntegrate;
  regionIntegrate.histogram_.setInfluenceRadius(influenceRadius);
  octree.integrateOptionalPayload<NormalPayload>(node->fixed().getMeanPosition(), radius, normalSamplingrate, regionIntegrate);

  return regionIntegrate.histogram_.calculateEntropy();
}

sure::Scalar sure::keypoints::calculateEntropyWithCrossproducts(const Octree& octree, Node* node, Scalar normalSamplingrate, Scalar radius, Scalar influenceRadius)
{
  FixedPayload mainNormalIntegrate;
  octree.integratePayload(node->fixed().getMeanPosition(), radius, mainNormalIntegrate);
  Normal mainNormal = mainNormalIntegrate.calculateNormal();

  if( mainNormal.isStable() )
  {
    sure::normal::CrossProductHistogram histogram;
    histogram.setInfluenceRadius(influenceRadius);
    NodeVector nodes = octree.getNodes(node->fixed().getMeanPosition(), radius, normalSamplingrate);

    for(unsigned int i=0; i<nodes.size(); ++i)
    {
      Node* currNode = nodes[i];
      NormalPayload* currPayload = static_cast<CrossProductPayload*>(currNode->opt());
      if( currPayload->normal_.isStable())
      {
        histogram.insertCrossProduct(mainNormal.vector(), currPayload->normal_.vector());
      }
    }

    return histogram.calculateEntropy();
  }
  return 0.0;
}

sure::Scalar sure::keypoints::calculateEntropyWithCrossproductsPairwise(const Octree& octree, Node* node, Scalar normalSamplingrate, Scalar radius, Scalar influenceRadius)
{
  sure::normal::CrossProductHistogram histogram;
  histogram.setInfluenceRadius(influenceRadius);
  NodeVector nodes = octree.getNodes(node->fixed().getMeanPosition(), radius, normalSamplingrate);
  for(unsigned int i=0; i<nodes.size(); ++i)
  {
    Node* firstNode = nodes[i];
    NormalPayload* firstPayload = static_cast<NormalPayload*>(firstNode->opt());
    const Normal& firstNormal = firstPayload->normal_;

    if( !firstNormal.isStable() )
    {
      continue;
    }

    for(unsigned int j=i+1; j<nodes.size(); ++j)
    {
      Node* secondNode = nodes[j];
      CrossProductPayload* secondPayload = static_cast<CrossProductPayload*>(secondNode->opt());
      const Normal& secondNormal = secondPayload->normal_;
      if( secondNormal.isStable() )
      {
        histogram.insertCrossProduct(firstNormal.vector(), secondNormal.vector());
      }
    }

    return histogram.calculateEntropy();
  }
  return 0.0;
}


sure::Scalar sure::keypoints::calculateCornerness(const Octree& octree, Node* node, Scalar radius)
{
  Octree::NodeVector vec;
  vec = octree.getNodes(node, octree.getUnitSize(radius));

  Vector3 mean(Vector3::Zero());
  Scalar weight(0.0);
  for(unsigned int i=0; i<vec.size(); ++i)
  {
    Node* currNode = vec[i];
    sure::payload::EntropyPayload* payload = static_cast<EntropyPayload*>(currNode->opt());
    if( payload->entropy_ > 0.0 )
    {
      mean += (payload->entropy_ * currNode->fixed().getMeanPosition());
//      mean[0] += (payload->entropy_ * currNode->fixed().getMeanPosition()[0]);
//      mean[1] += (payload->entropy_ * currNode->fixed().getMeanPosition()[1]);
//      mean[2] += (payload->entropy_ * currNode->fixed().getMeanPosition()[2]);
      weight += payload->entropy_;
    }
  }

  if( weight > 0.0 )
  {
    mean /= weight;
  }
  else
  {
    return 0.f;
  }

  Matrix3 covariance(Matrix3::Zero());
  for(unsigned int i=0; i<vec.size(); ++i)
  {
    Node* currNode = vec[i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(currNode->opt());

    if( payload->entropy_ > 0.0 )
    {
//      Vector3 d;
//      d[0] = mean[0] - currNode->fixed().getMeanPosition()[0];
//      d[1] = mean[1] - currNode->fixed().getMeanPosition()[1];
//      d[2] = mean[2] - currNode->fixed().getMeanPosition()[2];
      covariance += payload->entropy_ * ( (mean - currNode->fixed().getMeanPosition()) * ((mean - currNode->fixed().getMeanPosition()).transpose()) );
//      covariance += payload->entropy_ * ( d * d.transpose() );
    }
  }
  covariance /= weight;

  Vector3 eigenValues;
  pcl::eigen33(covariance, eigenValues);

  return (Scalar) (eigenValues[0] / eigenValues[2]);
}

void sure::keypoints::calculateCornerness(Octree& octree, Scalar samplingRate, Scalar radius, Scalar threshold)
{
  unsigned samplingDepth = octree.getDepth(samplingRate);

  for(unsigned int i=0; i<octree[samplingDepth].size(); ++i)
  {
    Node* currNode = octree[samplingDepth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(currNode->opt());

    if( payload->flag_ == POSSIBLE )
    {
      payload->cornerness_ = sure::keypoints::calculateCornerness(octree, currNode, radius);
      if( payload->cornerness_ < threshold )
      {
        payload->flag_ = CORNERNESS_TOO_LOW;
      }
    }
  }
}

unsigned sure::keypoints::extractKeypoints(Octree& octree, Scalar samplingrate, Scalar searchRadius, Scalar featureRadius, std::vector<Feature>& features, std::vector<Node*>& keypointNodes)
{
  unsigned samplingDepth = octree.getDepth(samplingrate);
  unsigned keypoints(0);

  for(unsigned int i=0; i<octree[samplingDepth].size(); ++i)
  {
    Node* currNode = octree[samplingDepth][i];
    EntropyPayload* payload = static_cast<EntropyPayload*>(currNode->opt());
    if( payload->flag_ != POSSIBLE )
    {
      continue;
    }
    NodeVector neighbors = octree.getNodes(currNode, octree.getUnitSize(searchRadius));
    for(unsigned int j=0; j<neighbors.size(); ++j)
    {
      Node* currNeighbor = neighbors[j];
      EntropyPayload* neighborPayload = static_cast<EntropyPayload*>(currNeighbor->opt());
      if( neighborPayload->flag_ == IS_MAXIMUM )
      {
        payload->flag_ = SUPPRESSED;
        break;
      }
      else if( neighborPayload->flag_ == POSSIBLE )
      {
        if( payload->entropy_ < neighborPayload->entropy_ )
        {
          payload->flag_ = SUPPRESSED;
          break;
        }
      }
      else
      {
        continue;
      }
    }
    if( payload->flag_ == POSSIBLE )
    {
      payload->flag_ = IS_MAXIMUM;
      sure::feature::Feature f;

      f.radius() = featureRadius;
      f.position() = currNode->fixed().getMeanPosition();
      features.push_back(f);
      keypointNodes.push_back(currNode);
      keypoints++;
    }
  }
  return keypoints;
}


void sure::keypoints::improveLocalization(const Octree& octree, Scalar samplingrate, Scalar radius, Vector3& position)
{
  for(unsigned iteration=0; iteration<NUMBER_OF_MEAN_SHIFT_ITERATIONS; ++iteration)
  {
    NodeVector neighbors = octree.getNodes(position, radius, samplingrate);

    Scalar summedMean(0.0);
    int count(0);

    for(unsigned int n=0; n<neighbors.size(); ++n)
    {
      Node* neighbor = neighbors[n];
      EntropyPayload* neighborPayload = static_cast<EntropyPayload*>(neighbor->opt());
      if( neighbor->fixed().getPointFlag() != ARTIFICIAL )
      {
        summedMean += neighborPayload->entropy_;
        count++;
      }
    }

    if( count == 0 )
    {
      break;
    }

    Scalar mean = summedMean / (Scalar) count;
    Scalar summedVariance(0.0);
    for(unsigned int n=0; n<neighbors.size(); ++n)
    {
      Node* neighbor = neighbors[n];
      EntropyPayload* neighborPayload = static_cast<EntropyPayload*>(neighbor->opt());
      if( neighbor->fixed().getPointFlag() != ARTIFICIAL )
      {
        summedVariance += (neighborPayload->entropy_ - mean) * (neighborPayload->entropy_ - mean);
      }
    }


    Scalar variance = summedVariance / (Scalar) count;
    Vector3 shiftedPosition(0.0, 0.0, 0.0);
    Scalar summedShift(0.0);

    for(unsigned int n=0; n<neighbors.size(); ++n)
    {
      Node* neighbor = neighbors[n];
      EntropyPayload* neighborPayload = static_cast<EntropyPayload*>(neighbor->opt());

      if( neighborPayload->entropy_ > mean )
      {
        Scalar entropyDifference = mean - neighborPayload->entropy_;
        Scalar kernelElements = (entropyDifference*entropyDifference) / (variance);
        shiftedPosition += neighbor->fixed().getMeanPosition() * exp(-0.5*kernelElements);
        summedShift += exp(-0.5*kernelElements);
      }
    }
    if( summedShift != 0 )
    {
      shiftedPosition = shiftedPosition * (1.f / summedShift);
    }
    if( sure::eigen_is_finite(shiftedPosition) )
    {
//      Scalar dist = (position - shiftedPosition).norm();
      position = shiftedPosition;
    }
  }
}

void sure::keypoints::improveLocalization(const Octree& octree, Scalar samplingrate, Scalar radius, std::vector<sure::feature::Feature>& features)
{
  for(std::vector<sure::feature::Feature>::iterator it=features.begin(); it!=features.end(); ++it)
  {
    sure::keypoints::improveLocalization(octree, samplingrate, radius, (*it).position());
  }
}


int sure::keypoints::removeRedundantKeypoints(Scalar searchRadius, std::vector<Feature>& features, std::vector<Node*>& keypointNodes)
{
  std::vector<bool> keypointStable(features.size(), true);
  Scalar radiusSquared = (searchRadius) * (searchRadius);
  for(unsigned firstIndex=0; firstIndex<features.size(); ++firstIndex)
  {
    std::deque<unsigned> conflicts;
    const Feature& firstKeypoint = features[firstIndex];
    const Vector3& firstPosition = firstKeypoint.position();

    for(unsigned secondIndex=firstIndex+1; secondIndex<features.size(); ++secondIndex)
    {
      const Feature& secondKeypoint = features[secondIndex];
      const Vector3& secondPosition = secondKeypoint.position();
      if( (firstPosition - secondPosition).squaredNorm() < radiusSquared )
      {
        conflicts.push_back(secondIndex);
      }
    }

    unsigned bestKeypoint = firstIndex;
    Scalar bestEntropy = static_cast<EntropyPayload*>(keypointNodes[firstIndex]->opt())->entropy_;

    for(unsigned j=0; j<conflicts.size(); ++j)
    {
      const unsigned& secondIndex = conflicts[j];
      const Feature& secondKeypoint = features[secondIndex];
      Scalar secondEntropy = static_cast<EntropyPayload*>(keypointNodes[secondIndex]->opt())->entropy_;
      if( secondEntropy > bestEntropy )
      {
        keypointStable[bestKeypoint] = false;
        bestKeypoint = secondIndex;
        bestEntropy = secondEntropy;
        break;
      }
      else
      {
        keypointStable[secondIndex] = false;
      }
    }
  }

  std::vector<Feature> clearedKeypoints;
  clearedKeypoints.reserve(features.size());

  for(unsigned i=0; i<keypointStable.size(); ++i)
  {
    const Feature& keypoint = features[i];
    if( !keypointStable[i] )
    {
      static_cast<EntropyPayload*>(keypointNodes[i]->opt())->flag_ = REDUNDANT;
      continue;
    }
    clearedKeypoints.push_back(keypoint);
  }
  int redundantFeatures = features.size() - clearedKeypoints.size();
  features = clearedKeypoints;
  return redundantFeatures;
}
