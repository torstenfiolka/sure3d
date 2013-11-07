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

template <typename FixedPayloadT>
std::ostream& sure::octree::operator<<(std::ostream& stream, const sure::octree::Octree<FixedPayloadT>& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(3);
  stream << "Octree center " << rhs.getCenter()[0] << "/" << rhs.getCenter()[1] << "/" << rhs.getCenter()[2] << "\n";
  stream << "Minimum volume size: " << rhs.minimumNodeSize_ << " minimum internal volume size: " << rhs.maxNodeResolution_ << "\n";
  if( rhs.root_ )
  {
    stream << "Units: " << rhs.root_->region().dimensions() << " - Expansion: " << rhs.getSize(rhs.root_->region()) << "m - maximum depth: " << rhs.getMaximumDepth() << "\n";
    for(unsigned int i=0; i<=rhs.getMaximumDepth(); ++i)
    {
      stream << "Depth: " << i << " number of nodes: " << rhs.map_.at(i).size() << " size: " << rhs.getUnitSizeFromDepth(i) << " units (" << rhs.getSizeFromDepth(i) << "m)\n";
    }
  }
  else
  {
    stream << "No root\n";
  }
  stream << rhs.allocator_;
  stream.unsetf(std::ios_base::fixed);
  return stream;
}

template <typename FixedPayloadT>
void sure::octree::Octree<FixedPayloadT>::clear()
{
  allocator_.clear();
  root_ = NULL;
  maxDepth_ = 0;
  minimumNodeSize_ = DEFAULT_MINIMUM_NODE_SIZE;
  octreeCenter_ = Vector3::Zero();
  map_.clear();
  initialized_ = false;
}

template <typename FixedPayloadT>
typename sure::octree::Octree<FixedPayloadT>::NodeVector sure::octree::Octree<FixedPayloadT>::getNodes(const Region& r) const
{
  NodeVector v;
  std::deque<Node*> nodeList;
  nodeList.push_back(root_);
  Node* current;
  while( !nodeList.empty() )
  {
    current = nodeList.front();
    nodeList.pop_front();
    if( r.contains(current->region()) )
    {
      v.push_back(current);
    }
    else if( current->depth() < maxDepth_-1 )
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && r.overlaps(current->children_[i]->region()) )
        {
          nodeList.push_back(current->children_[i]);
        }
      }
    }
    else
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && r.overlaps(current->children_[i]->region()) )
        {
          v.push_back(current->children_[i]);
        }
      }
    }
  }
  return v;
}

template <typename FixedPayloadT>
typename sure::octree::Octree<FixedPayloadT>::NodeVector sure::octree::Octree<FixedPayloadT>::getNodes(const Region& r, unsigned depth) const
{
  NodeVector v;
  std::deque<Node*> nodeList;
  nodeList.push_back(root_);
  Node* current;
  while( !nodeList.empty() )
  {
    current = nodeList.front();
    nodeList.pop_front();
    unsigned currDepth = current->depth();

    if( currDepth < depth-1 )
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && current->children_[i]->region().overlaps(r) )
        {
          nodeList.push_back(current->children_[i]);
        }
      }
    }
    else
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && current->children_[i]->region().overlaps(r) )
        {
          v.push_back(current->children_[i]);
        }
      }
    }
  }
  return v;
}

template <typename FixedPayloadT>
unsigned sure::octree::Octree<FixedPayloadT>::integratePayload(const Region& r, FixedPayloadT& payload) const
{
  unsigned count(0);

  std::deque<Node*> nodeList;
  nodeList.push_back(root_);
  Node* current;
  while( !nodeList.empty() )
  {
    current = nodeList.front();
    nodeList.pop_front();
    if( r.contains(current->region_) )
    {
      payload += current->fixed_;
      count++;
    }
    else if( current->depth() < maxDepth_-1 )
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && r.overlaps(current->children_[i]->region()) )
        {
          nodeList.push_back(current->children_[i]);
        }
      }
    }
    else
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && r.overlaps(current->children_[i]->region()) )
        {
          payload += current->children_[i]->fixed();
          count++;
        }
      }
    }
  }
  return count;
}

template <typename FixedPayloadT>
unsigned sure::octree::Octree<FixedPayloadT>::integratePayload(const Region& r, unsigned depth, FixedPayloadT& payload) const
{
  unsigned count(0);

  std::deque<Node*> nodeList;
  nodeList.push_back(root_);
  Node* current;
  while( !nodeList.empty() )
  {
    current = nodeList.front();
    nodeList.pop_front();
    unsigned currDepth = current->depth();

    if( currDepth < depth-1 )
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && current->children_[i]->region().overlaps(r) )
        {
          nodeList.push_back(current->children_[i]);
        }
      }
    }
    else
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && current->children_[i]->region().overlaps(r) )
        {
          payload += current->fixed_;
          count++;
        }
      }
    }
  }
  return count;
}

template <typename FixedPayloadT>
template <typename OptionalPayloadT>
unsigned sure::octree::Octree<FixedPayloadT>::integrateOptionalPayload(const Region& r, OptionalPayloadT& payload) const
{
  unsigned count(0);

  std::deque<Node*> nodeList;
  nodeList.push_back(root_);
  Node* current;
  while( !nodeList.empty() )
  {
    current = nodeList.front();
    nodeList.pop_front();
    if( r.contains(current->region_) )
    {
      if( current->opt() )
      {
        payload += *(static_cast<OptionalPayloadT*>(current->opt()));
        count++;
      }
    }
    else if( current->depth() < maxDepth_-1 )
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && r.overlaps(current->children_[i]->region()) )
        {
          nodeList.push_back(current->children_[i]);
        }
      }
    }
    else
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && r.overlaps(current->children_[i]->region()) )
        {
          if( current->children_[i]->opt() )
          {
            payload += *(static_cast<OptionalPayloadT*>(current->children_[i]->opt()));
            count++;
          }
        }
      }
    }
  }
  return count;
}

template <typename FixedPayloadT>
template <typename OptionalPayloadT>
unsigned sure::octree::Octree<FixedPayloadT>::integrateOptionalPayload(const Region& r, unsigned depth, OptionalPayloadT& payload) const
{
  unsigned count(0);

  std::deque<Node*> nodeList;
  nodeList.push_back(root_);
  Node* current;
  while( !nodeList.empty() )
  {
    current = nodeList.front();
    nodeList.pop_front();
    unsigned currDepth = current->depth();

    if( currDepth < depth-1 )
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && current->children_[i]->region().overlaps(r) )
        {
          nodeList.push_back(current->children_[i]);
        }
      }
    }
    else
    {
      for(unsigned i=0; i<OCTANT; ++i)
      {
        if( current->children_[i] && current->children_[i]->region().overlaps(r) )
        {
          if( current->children_[i]->opt() )
          {
            payload += *(static_cast<OptionalPayloadT*>(current->children_[i]->opt()));
            count++;
          }
        }
      }
    }
  }
  return count;
}

/**
 * Returns a pointer to the node which contains the given Address on a given level. If level ist not set or zero, the maximum level will be assumed.
 * If no node contains the Address, NULL will be returned.
 */
template <typename FixedPayloadT>
typename sure::octree::Octree<FixedPayloadT>::Node* sure::octree::Octree<FixedPayloadT>::getNode(const Point& a, unsigned depth) const
{
  Node* currNode(root_);
  unsigned currDepth(0);
  if( depth == 0 )
  {
    depth = maxDepth_;
  }
  if( !currNode->range().contains(a) )
  {
    currNode = NULL;
  }
  while( currNode && currDepth < depth )
  {
    OctantType oct = currNode->range().getOctant(a);
    currNode = currNode->children_[oct];
    currDepth++;
  }
  return currNode;
}

/**
 * Returns a pointer to the node nearest to the given Address on a given level. If level ist not set or zero, the maximum level will be assumed.
 *
 */
template <typename FixedPayloadT>
typename sure::octree::Octree<FixedPayloadT>::Node* sure::octree::Octree<FixedPayloadT>::getNextNode(const Point& a, unsigned depth) const
{
  Node* currNode(root_);
  unsigned radius(DEFAULT_MIN_NODE_UNIT_RADIUS);
  if( depth == 0 )
  {
    depth = maxDepth_;
  }
  if( currNode->region().contains(a) )
  {
    while( radius < root_->region().size() && currNode == root_ )
    {
      NodeVector nodes = getNodes(a, radius, depth);
      unsigned bestDistance = root_->region().size();
      for(typename NodeVector::iterator it=nodes.begin(); it!=nodes.end(); ++it)
      {
        unsigned d = ((*it)->center() - a).squaredNorm();
        if( d < bestDistance )
        {
          currNode = *it;
          bestDistance = d;
        }
      }
      radius+=getUnitSizeFromDepth(depth);
    }
  }
  return currNode;
}


template <typename FixedPayloadT>
bool sure::octree::Octree<FixedPayloadT>::initialize(Scalar minNodeSize, Scalar expansion, unsigned capacity, const Vector3& center)
{
  clear();

  minimumNodeSize_ = minNodeSize;
  octreeCenter_ = center;

  unsigned dimension;
  dimension = (expansion / minimumNodeSize_) * (float) DEFAULT_MIN_NODE_UNIT_SIZE;
  if( dimension < DEFAULT_MIN_NODE_UNIT_SIZE )
  {
    return initialized_;
  }
  if( dimension >= 8 && dimension % 8 != 0 )
  {
    return initialized_;
  }

  maxDepth_ = 0;
  while( fabs(expansion - minimumNodeSize_) > std::numeric_limits<float>::epsilon() )
  {
    expansion *= 0.5f;
    maxDepth_++;
  }

  allocator_.resize(capacity);

  root_ = NULL;
  root_ = allocator_.allocate();
  if( !root_ )
  {
    return initialized_;
  }
  root_->region_ = Region(Point(0, 0, 0), dimension/2);

  map_.clear();
  map_[0].push_back(root_);

  initialized_ = true;
  return initialized_;
}

template <>
template <typename PointT>
void sure::octree::Octree<sure::payload::PointsRGB>::addPointCloud(const pcl::PointCloud<PointT>& cloud)
{
  if( cloud.size() == 0 )
  {
    std::cout << "Pointcloud empty, skipping octree building.\n";
    return;
  }
  for(unsigned int i=0; i<cloud.size(); ++i)
  {
    const PointT& p = cloud.at(i);
    if( std::isfinite(p.x) )
    {
      Point a(getAddress(p.x, p.y, p.z));
      Region r(a, DEFAULT_MIN_NODE_UNIT_RADIUS);
      Node n(r);
      n.fixed().setPosition(p.x, p.y, p.z);
      n.fixed().setColor(p.rgb);
      n.fixed().setFlag(NORMAL);
      insertNode(root_, n, 0);
    }
  }
}

template <>
template <typename PointT>
void sure::octree::Octree<sure::payload::PointsRGB>::addPointCloud(const pcl::PointCloud<PointT>& cloud, const sure::range_image::RangeImage<PointT>& rangeImage)
{
  if( cloud.size() == 0 )
  {
    std::cout << "Pointcloud empty, skipping octree building.\n";
    return;
  }
  for(unsigned int i=0; i<cloud.size(); ++i)
  {
    const PointT& p = cloud.at(i);
    if( std::isfinite(p.x) )
    {
      Point a(getAddress(p.x, p.y, p.z));
      Region r(a, DEFAULT_MIN_NODE_UNIT_RADIUS);
      Node n(r);
      n.fixed().setPosition(p.x, p.y, p.z);
      n.fixed().setColor(p.rgb);
      n.fixed().setFlag(NORMAL);
      if( rangeImage.isBackgroundBorder(i) )
      {
        n.fixed().setFlag(BACKGROUND_BORDER);
      }
      if( rangeImage.isForegroundBorder(i) )
      {
        n.fixed().setFlag(FOREGROUND_BORDER);
      }
      insertNode(root_, n, 0);
    }
  }
}

template <>
template <typename PointT>
void sure::octree::Octree<sure::payload::PointsRGB>::addArtificialPointCloud(const pcl::PointCloud<PointT>& cloud)
{
  if( cloud.size() == 0 )
  {
    std::cout << "Pointcloud empty, skipping octree building.\n";
    return;
  }
  for(unsigned int i=0; i<cloud.size(); ++i)
  {
    const PointT& p = cloud.at(i);
    if( std::isfinite(p.x) )
    {
      Point a(getAddress(p.x, p.y, p.z));
      Region r(a, DEFAULT_MIN_NODE_UNIT_RADIUS);
      Node n(r);
      n.fixed().setPosition(p.x, p.y, p.z);
      n.fixed().setColor(p.rgb);
      n.fixed().setFlag(ARTIFICIAL);
      insertNode(root_, n, 0);
    }
  }
}

template <typename FixedPayloadT>
void sure::octree::Octree<FixedPayloadT>::insertNode(Node* current, const Node& node, unsigned level)
{
  current->fixed_ += node.fixed_;

  if( level < maxDepth_ )
  {
    OctantType octant = current->region_.getOctant(node.region_.center());
    if( !current->children_[octant] )
    {
      current->children_[octant] = allocator_.allocate();
      current->children_[octant]->region_ = current->region_.getOctant(octant);
      current->children_[octant]->parent_ = current;
      map_[level+1].push_back(current->children_[octant]);
    }
    level++;
    insertNode(current->children_[octant], node, level);
  }
}
