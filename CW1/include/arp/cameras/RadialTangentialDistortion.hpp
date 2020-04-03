/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file cameras/RadialTangentialDistortion.hpp
 * @brief Header file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CAMERAS_RADIALTANGENTIALDISTORTION_HPP_
#define INCLUDE_OKVIS_CAMERAS_RADIALTANGENTIALDISTORTION_HPP_

#include <memory>
#include <Eigen/Core>
#include "arp/cameras/DistortionBase.hpp"

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

class RadialTangentialDistortion : public DistortionBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The default constructor with all zero ki
  inline RadialTangentialDistortion();

  /// \brief Constructor initialising ki
  /// @param[in] k1 radial parameter 1
  /// @param[in] k2 radial parameter 2
  /// @param[in] p1 tangential parameter 1
  /// @param[in] p2 tangential parameter 2
  inline RadialTangentialDistortion(double k1, double k2, double p1, double p2);

  /// \brief Unit test support -- create a test distortion object
  static RadialTangentialDistortion testObject()
  {
    return RadialTangentialDistortion(-0.16, 0.15, 0.0003, 0.0002);
  }

  //////////////////////////////////////////////////////////////
  /// \name Distortion functions
  /// @{

  /// \brief Distortion only
  /// @param[in]  pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointDistorted   The distorted normalised (!) image point.
  /// @return     True on success (no singularity)
  inline bool distort(const Eigen::Vector2d & pointUndistorted,
                      Eigen::Vector2d * pointDistorted) const;

  /// \brief Distortion and Jacobians.
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the intrinsics vector.
  /// @return     True on success (no singularity)
  inline bool distort(const Eigen::Vector2d & pointUndistorted,
                      Eigen::Vector2d * pointDistorted,
                      Eigen::Matrix2d * pointJacobian) const;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistortion functions
  /// @{

  /// \brief Undistortion only
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @return     True on success (no singularity)
  inline bool undistort(const Eigen::Vector2d & pointDistorted,
                        Eigen::Vector2d * pointUndistorted) const;
  /// @}

 protected:
  double k1_;  ///< radial parameter 1
  double k2_;  ///< radial parameter 2
  double p1_;  ///< tangential parameter 1
  double p2_;  ///< tangential parameter 2
};

}  // namespace cameras
}  // namespace arp

#include "implementation/RadialTangentialDistortion.hpp"

#endif /* INCLUDE_OKVIS_CAMERAS_RADIALTANGENTIALDISTORTION_HPP_ */
