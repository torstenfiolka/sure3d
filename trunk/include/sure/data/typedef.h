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

#ifndef SURE_TYPEDEF_H_
#define SURE_TYPEDEF_H_

#include <Eigen/Dense>

namespace sure
{

  /**
   * Defines the mode for entropy calculation:
   * NORMALS: Discretizes normals in a histogram and calculates the entropy on the histogram
   * CROSS_PRODUCTS_W_MAIN: Calculates the cross-products between all normals in a region and the main normal
   * of the region and uses the discretized cross-products for entropy calculation
   * CROSS_PRODUCTS_PAIRWISE: Calculates cross-products pairwise between all normals in a region and uses the
   * discretized cross-products for entropy calculation
   */
  enum EntropyCalculationMode
  {
    NORMALS = 0,
    CROSS_PRODUCTS_W_MAIN,
    CROSS_PRODUCTS_PAIRWISE
  };

  /**
   * Flags regarding the points contained in a node
   */
  enum PointFlag
  {
    NO_FLAG = 0,          //!< NO_FLAG Default value
    NORMAL = 1,           //!< NORMAL Valid points from a point cloud
    ARTIFICIAL = 2,       //!< ARTIFICIAL Artificial points
    BACKGROUND_BORDER = 4,//!< BACKGROUND_BORDER Points from a background border
    FOREGROUND_BORDER = 8 //!< FOREGROUND_BORDER Points from a foreground border
  };

  /**
   * Flags concerning feature calculation
   */
  enum MaximumFlag
  {
    IS_MAXIMUM = 0,    //!< IS_MAXIMUM Node is an entropy maximum
    POSSIBLE,          //!< POSSIBLE Node may be an entropy maximum
    NOT_CALCULATED,    //!< NOT_CALCULATED No entropy calculation has been made
    ENTROPY_TOO_LOW,   //!< ENTROPY_TOO_LOW Entropy lower than threshold
    CORNERNESS_TOO_LOW,//!< CORNERNESS_TOO_LOW Cornerness lower than threshold
    SUPPRESSED,        //!< SUPPRESSED Node is suppressed by neighboring node with higher entropy
    REDUNDANT,         //!< REDUNDANT Two maxima are too close together after improving localization
    DISTANCE_TOO_HIGH, //!< DISTANCE_TOO_HIGH Distance from sensor is too high
    BACKGROUND_EDGE,   //!< BACKGROUND_EDGE Node lies on a background border
    ARTIFICIAL_POINTS, //!< ARTIFICIAL_POINTS Node consists of artificial points ONLY
  };


  // ********
  // Typedefs
  // ********

  // Default types
  typedef double Scalar;
  typedef unsigned char OctantType;
  const OctantType OCTANT = 8;

  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::Vector3d NormalType;
  typedef Eigen::aligned_allocator<NormalType> NormalTypeAllocator;

  // Default type for addressing regions used in the octree
  typedef int PointUnit;
  const PointUnit ZERO = 0;

  // Default type for histograms
  typedef float HistoType;


  // **********************
  // Some general functions
  // **********************

  //! Tests an Eigen Type for infinity and NaN
  template<typename Derived>
  bool eigen_is_finite(const Eigen::MatrixBase<Derived>& x)
  {
     return ( (x - x).array() == (x - x).array()).all();
  }

  const int getNormalHistogramSize(unsigned numberOfPolarBins);

  const Scalar getNormalHistogramBinSize(unsigned numberOfPolarBins);

  // *********
  // Constants
  // *********

  static const float EPSILON = 1e-4;

  namespace descriptor
  {
    //! Cap for the Earth Mover's distance
    const double MAX_EARTH_MOVERS_DISTANCE = 2.0;

    //! Color descriptor constants
    const int COLOR_DESCRIPTOR_SIZE = 24;
    const int EXTRA_BIN_FOR_SATURATION_BALANCE = 1;
    const HistoType MIN_HUE = 0.0;
    const HistoType MAX_HUE = 6.0;
    const HistoType MIN_SATURATION = 0.0;
    const HistoType MAX_SATURATION = 1.0;

    //! Shape descriptor constants
    const int SHAPE_DESCRIPTOR_SIZE = 11;
    const HistoType SHAPE_HISTOGRAM_VALUE = 1.0;

    const HistoType ALPHA_MIN = -1.0;
    const HistoType ALPHA_MAX = 1.0;
    const HistoType PHI_MIN = -1.0;
    const HistoType PHI_MAX = 1.0;
    const HistoType THETA_MIN = -M_PI;
    const HistoType THETA_MAX = M_PI;

    //! Lightness descriptor constants
    const int LIGHTNESS_DESCRIPTOR_SIZE = 10;
    // lightness is stored relative to the descriptor's center lightness, so it can be nagative
    const HistoType MIN_LIGHTNESS = -1.0;
    const HistoType MAX_LIGHTNESS = 1.0;

  }
  namespace range_image
  {
    const Vector3 VIEW_AXIS = Vector3::UnitZ();
    const Scalar MAX_USED_POINT_DISTANCE = 7.5f;
  }
  namespace normal
  {
    const unsigned MIN_NUMBER_OF_POINTS_FOR_STABLE_NORMAL = 5;

    /**
     * Number of splits along the polar angle
     * The histogram size depends on the value
     * 4:22 5:34 6:49 7:66 10:134
     */
    const int NORMAL_HISTOGRAM_POLAR_SPLITS = 5;
    static const int NORMAL_HISTOGRAM_SIZE = 34;
    const int CROSS_PRODUCT_HISTOGRAM_SIZE = NORMAL_HISTOGRAM_SIZE+1;
    const Scalar AVG_NORMAL_HISTOGRAM_BIN_SIZE = getNormalHistogramBinSize(NORMAL_HISTOGRAM_POLAR_SPLITS);
    const Scalar DEFAULT_NORMAL_HISTOGRAM_INFLUENCE = AVG_NORMAL_HISTOGRAM_BIN_SIZE * 1.5;

    const HistoType LOG_BASE_2 = log(2.f);
  }
  namespace feature
  {
    const unsigned DEFAULT_NUMBER_OF_DESCRIPTORS = 2;
  }
  namespace octree
  {
    const unsigned DEFAULT_MIN_NODE_UNIT_RADIUS = 1;
    const unsigned DEFAULT_MIN_NODE_UNIT_SIZE = 2 * DEFAULT_MIN_NODE_UNIT_RADIUS;
    const Scalar DEFAULT_MINIMUM_NODE_SIZE = 0.01;
  }
}


#endif /* SURE_TYPEDEF_H_ */
