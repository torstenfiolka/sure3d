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

using sure::HistoType;

template <int SizeT>
std::ostream& sure::descriptor::operator<<(std::ostream& stream, const Histogram<SizeT>& rhs)
{
  stream.setf(std::ios_base::fixed);
  stream << std::setprecision(2);
  stream << "Histogram - minimum value: " << rhs.min_ << " - maximum value: " << rhs.max_ << " - bin size: " << rhs.binSize_ << " - stored weight: " << rhs.weight_ << "\nValues:";
  for(int i=0; i<SizeT; ++i)
  {
    stream << " " << rhs.array_[i];
  }
  stream << "\n";
  stream.unsetf(std::ios_base::fixed);
  return stream;
}


template<int SizeT>
void sure::descriptor::Histogram<SizeT>::insertSmooth(HistoType value, HistoType weight, bool circumferential)
{
  if( value < min_ || value > max_ )
  {
    return;
  }
  int targetBin = floor(((value - min_) / binSize_));
  HistoType insertedWeights = 0.0;

  for(int i=targetBin-1; i<=targetBin+1; ++i)
  {
    int currIndex = i;
    if( currIndex < 0 )
    {
      if( circumferential )
      {
        currIndex += SizeT;
      }
      else
      {
        continue;
      }
    }
    if( currIndex >= (int) SizeT )
    {
      if( circumferential )
      {
        currIndex -= SizeT;
      }
      else
      {
        continue;
      }
    }
    HistoType distance = fabs((value - min_) - ((0.5 + (HistoType) currIndex) * binSize_ ));
    if( distance < binSize_ )
    {
      array_[currIndex] += weight * (distance / binSize_);
      insertedWeights += weight * (distance / binSize_);
    }
  }
  weight_ += insertedWeights;
}

template<int SizeT>
void sure::descriptor::Histogram<SizeT>::insertPrecise(HistoType value, HistoType weight)
{
  if( value < min_ || value > max_ )
  {
    return;
  }

  int targetBin = floor(((value - min_) / binSize_));

  if( targetBin < 0 || targetBin >= SizeT )
  {
    return;
  }

  array_[targetBin] += weight;
  weight_ += weight;
}


template<int SizeT>
HistoType sure::descriptor::Histogram<SizeT>::EMDistance(const sure::descriptor::Histogram<SizeT>& rhs, const std::vector<std::vector<double> >& distanceMatrix) const
{
  HistoType distance = 0.0;
  std::vector<double> lhsVec, rhsVec;
  this->fillVector(lhsVec);
  rhs.fillVector(rhsVec);
  distance += emd_hat<double>()(rhsVec, lhsVec, distanceMatrix);
  return (distance / (HistoType) MAX_EARTH_MOVERS_DISTANCE);
}

template<int SizeT>
HistoType sure::descriptor::Histogram<SizeT>::L2Distance(const Histogram<SizeT>& rhs) const
{
  if( this->weight_ > 0.0 || rhs.weight_ > 0.0 )
  {
    HistoType distance(0.0);
    for(int i=0; i<SizeT; ++i)
    {
      distance += (rhs[i] - this->array_[i]) * (rhs[i] - this->array_[i]);
    }
    return sqrt(distance) / (this->weight_+rhs.weight_);
  }
  return INFINITY;
}

template<int SizeT>
void sure::descriptor::Histogram<SizeT>::fillVector(std::vector<double>& vec) const
{
  if( weight_ > 0.0 )
  {
    vec.resize(SizeT);
    if( weight_ == 1.0 )
    {
      for(unsigned i=0; i<SizeT; ++i)
      {
        vec[i] = array_[i];
      }
    }
    else
    {
      for(unsigned i=0; i<SizeT; ++i)
      {
        vec[i] = array_[i] / weight_;
      }
    }
  }
  else
  {
    vec.resize(SizeT, 0.0);
  }
}

template<int SizeT>
void sure::descriptor::Histogram<SizeT>::fillHistogramRandom(int seed)
{
  this->clear();
  if( seed > 0 )
  {
    srand(seed);
  }
  for(unsigned j=0; j<SizeT; ++j)
  {
    this->array_[j] = (HistoType) rand() / (HistoType) RAND_MAX;
    this->weight_ += this->array_[j];
  }
  this->normalize();
}


//! creates the earth mover's distance matrix for the lightness and color descriptor
template <int SizeT>
std::vector<std::vector<double> > sure::descriptor::initEMDMatrix(double maxDistance)
{
  std::vector<std::vector<double> > matrix;
  matrix.resize(SizeT);
  for(unsigned i=0; i<SizeT; ++i)
  {
    matrix[i].resize(SizeT);
    for(unsigned j=0; j<SizeT; ++j)
    {
      matrix[i][j] = std::min((double) abs(i-j), maxDistance);
    }
  }
  return matrix;
}

