// Software License Agreement (BSD License)
//
// Copyright (c) 2012, Fraunhofer FKIE/US
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

template<unsigned int histoSize>
const std::string sure::DescriptorHistogramWithEMDistance<histoSize>::HISTOGRAM_NAME = std::string("EMD Histogram");

template<unsigned int histoSize>
const double sure::DescriptorHistogramWithEMDistance<histoSize>::MAX_DISTANCE = 2.0;

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize>& sure::DescriptorHistogramWithEMDistance<histoSize>::operator=(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs)
{
  if( this != &rhs )
  {
    sure::HistogramBase::operator =(rhs);
    this->histogram = rhs.histogram;
  }
  return *this;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize>& sure::DescriptorHistogramWithEMDistance<histoSize>::operator=(const sure::HistogramBase& rhs)
{
  if( this != &rhs )
  {
    sure::HistogramBase::operator =(rhs);
  }
  return *this;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize> sure::DescriptorHistogramWithEMDistance<histoSize>::operator +(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs) const
{
  sure::DescriptorHistogramWithEMDistance<histoSize> lhs(*this);
  lhs = ((sure::HistogramBase) lhs).operator +=(rhs);
  for(unsigned int i=0; i<lhs.histogramSize; ++i)
  {
    lhs.histogram[i] += rhs.histogram[i];
  }
  return lhs;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize>& sure::DescriptorHistogramWithEMDistance<histoSize>::operator +=(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs)
{
  *this = ((sure::HistogramBase) *this).operator +=(rhs);
  for(unsigned int i=0; i<histogramSize; ++i)
  {
    this->histogram[i] += rhs.histogram[i];
  }
  return *this;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize> sure::DescriptorHistogramWithEMDistance<histoSize>::operator *(const double rhs) const
{
  sure::DescriptorHistogramWithEMDistance<histoSize> lhs(*this);
  lhs = ((sure::HistogramBase) lhs).operator *=(rhs);
  for(unsigned int i=0; i<histogramSize; ++i)
  {
    lhs.histogram[i] *= rhs;
  }
  return lhs;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize>& sure::DescriptorHistogramWithEMDistance<histoSize>::operator *=(const double rhs)
{
  *this = ((sure::HistogramBase) *this).operator *=(rhs);
  for(unsigned int i=0; i<histogramSize; ++i)
  {
    this->histogram[i] *= rhs;
  }
  return *this;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize> sure::DescriptorHistogramWithEMDistance<histoSize>::operator /(const double rhs) const
{
  sure::DescriptorHistogramWithEMDistance<histoSize> lhs(*this);
  lhs = ((sure::HistogramBase) lhs).operator /=(rhs);
  for(unsigned int i=0; i<histogramSize; ++i)
  {
    lhs.histogram[i] /= rhs;
  }
  return lhs;
}

template<unsigned int histoSize>
sure::DescriptorHistogramWithEMDistance<histoSize>& sure::DescriptorHistogramWithEMDistance<histoSize>::operator /=(const double rhs)
{
  *this = ((sure::HistogramBase) *this).operator /=(rhs);
  for(unsigned int i=0; i<histogramSize; ++i)
  {
    this->histogram[i] /= rhs;
  }
  return *this;
}


template<unsigned int histoSize>
void sure::DescriptorHistogramWithEMDistance<histoSize>::clear()
{
  sure::HistogramBase::clear();
  histogram.clear();
  histogram.resize(histogramSize, 0.0);
}

template<unsigned int histoSize>
void sure::DescriptorHistogramWithEMDistance<histoSize>::normalize()
{
  if( histogramWeight > 0 )
  {
    for(unsigned int j=0; j<histogramSize; ++j)
    {
      histogram[j] = histogram[j] / (double) histogramWeight;
    }
    histogramWeight = 1.0;
  }
}

template<unsigned int histoSize>
void sure::DescriptorHistogramWithEMDistance<histoSize>::print() const
{
  sure::HistogramBase::print();
  double sum = 0.0;
  std::cout << "[EMD_Histogram] " << name << ": ";
  for(unsigned int j=0; j<histogramSize; ++j)
  {
    std::cout << std::fixed << std::setprecision(3) << histogram[j] << " ";
    sum += histogram[j];
  }
  std::cout << std::fixed << std::setprecision(3) << "Sum: " << sum << std::endl;
}

template<unsigned int histoSize>
void sure::DescriptorHistogramWithEMDistance<histoSize>::print(const std::vector<std::vector<double> >& distMatrix) const
{
  for(unsigned int i=0; i<distMatrix.size(); ++i)
  {
    std::cout << "[EMD_Distanzmatrix] " << name << ": ";
    for(unsigned int j=0; j<distMatrix.size(); ++j)
    {
      std::cout << std::fixed << std::setprecision(1) << distMatrix[i][j] << " ";
    }
    std::cout << std::endl;
  }
}


template<unsigned int histoSize>
void sure::DescriptorHistogramWithEMDistance<histoSize>::insertValue(double value)
{
  if( value < minimum || value > maximum )
  {
    return;
  }
  int targetBin = floor((value - minimum) / sure::DescriptorHistogramWithEMDistance<histoSize>::binSize);
  double distance, weight = 0.0;

  for(int i=targetBin-1; i<=targetBin+1; ++i)
  {
    if( i < 0 || i >= (int) histogramSize )
    {
      continue;
    }
    distance = fabs((value - minimum) - ((0.5 + (double) i) * sure::DescriptorHistogramWithEMDistance<histoSize>::binSize ));
    if( distance < sure::DescriptorHistogramWithEMDistance<histoSize>::binSize )
    {
      histogram[i] += (1.0 - (distance / sure::DescriptorHistogramWithEMDistance<histoSize>::binSize));
      weight += (1.0 - (distance / sure::DescriptorHistogramWithEMDistance<histoSize>::binSize));
    }
  }
  numberOfPoints++;
  histogramWeight += weight;
}

template<unsigned int histoSize>
double sure::DescriptorHistogramWithEMDistance<histoSize>::distanceTo(const sure::DescriptorHistogramWithEMDistance<histoSize>& rhs) const
{
  if( !this->isNormalized() || !rhs.isNormalized() )
  {
    return INFINITY;
  }
  double distance = 0.0;
  distance += emd_hat<double>()(this->histogram, rhs.histogram, this->DISTANCE_MATRIX);
  return (distance * (1.0 / (double) this->MAX_DISTANCE));
}

template<unsigned int histoSize>
void sure::DescriptorHistogramWithEMDistance<histoSize>::fillRandom()
{
  sure::HistogramBase::clear();
  for(unsigned int j=0; j<histogramSize; ++j)
  {
    histogram[j] = (double) rand() / (double) RAND_MAX;
    histogramWeight += histogram[j];
  }
  this->normalize();
}

//! creates the earth mover's distance matrix for the lightness and color descriptor
template<unsigned int histoSize>
std::vector<std::vector<double> > sure::initEMDMatrix()
{
  std::vector<std::vector<double> > matrix;
  matrix.resize(histoSize);
  for(unsigned int i=0; i<histoSize; ++i)
  {
    matrix[i].resize(histoSize);
    for(unsigned int j=0; j<histoSize; ++j)
    {
      matrix[i][j] = std::min((double) abs(i-j), sure::DescriptorHistogramWithEMDistance<histoSize>::MAX_DISTANCE);
    }
  }
  return matrix;
}

template<unsigned int histoSize>
const std::vector<std::vector<double> > sure::DescriptorHistogramWithEMDistance<histoSize>::DISTANCE_MATRIX = sure::initEMDMatrix<histoSize>();


