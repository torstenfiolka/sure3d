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

#include <sure/normal/normal_estimation.h>

void sure::normal::allocateNormalPayload(Octree& octree, Scalar samplingrate, sure::memory::FixedSizeAllocatorWithDirectAccess<NormalPayload>& allocator)
{
  unsigned depth = octree.getDepth(samplingrate);
  allocator.resizeIfSmaller(octree[depth].size());
  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* node = octree[depth][i];
    NormalPayload* payload = allocator.allocate(i);
    node->setOptionalPayload(static_cast<sure::payload::Payload*>(payload));
  }
}

void sure::normal::orientateNormal(const Vector3& pos, Normal& normal, const Vector3& orientation)
{
  if( normal.vector().dot(orientation-pos) < Scalar(0) )
  {
    normal.vector() = -normal.vector();
  }
}

bool sure::normal::estimateNormal(const Octree& octree, const Vector3& p, Scalar radius, Normal& normal)
{
  PointsRGB regionIntegrate;
  octree.integratePayload(p, radius, regionIntegrate);

  normal = regionIntegrate.calculateNormal();
  return normal.isStable();
}


unsigned sure::normal::estimateNormals(Octree& octree, Scalar samplingrate, Scalar radius, const Vector3& orientationPoint, Scalar histogramInfluence)
{
  unsigned count(0);
  unsigned depth = octree.getDepth(samplingrate);

  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* currNode = octree[depth][i];
    NormalPayload* payload = static_cast<NormalPayload*>(currNode->opt());

    if( payload->normal_.getStatus() != Normal::NORMAL_NOT_CALCULATED )
    {
      continue;
    }

    if( estimateNormal(octree, currNode->fixed().getMeanPosition(), radius, payload->normal_) )
    {
      orientateNormal(currNode->fixed().getMeanPosition(), payload->normal_, orientationPoint);

      payload->histogram_.setInfluenceRadius(histogramInfluence);
      payload->histogram_.insertNormal(payload->normal_);
      count++;
    }
  }
  return count;
}

unsigned sure::normal::discardNormalsfromNodesWithFlag(Octree& octree, Scalar samplingrate, PointFlag flag)
{
  unsigned count(0);
  unsigned depth = octree.getDepth(samplingrate);

  for(unsigned int i=0; i<octree[depth].size(); ++i)
  {
    Node* currNode = octree[depth][i];
    NormalPayload* payload = static_cast<NormalPayload*>(currNode->opt());

    if( (currNode->fixed().getPointFlag() | flag) == flag )
    {
      payload->normal_.setUnstable();
      count++;
    }
  }
  return count;
}
