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
 *  Created on: Jan 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file cameras/CameraBase.hpp
 * @brief Header file for the CameraBase class.
 * @author Stefan Leutenegger
 */


#ifndef INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_
#define INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_

#include <vector>
#include <memory>
#include <stdint.h>
#include <Eigen/Core>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include <arp/cameras/DistortionBase.hpp>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \class CameraBase
/// \brief Base class for all camera models.
class CameraBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \class ProjectionStatus
  /// \brief Indicates what happened when applying any of the project functions.
  enum class ProjectionStatus
  {
    Successful,
    OutsideImage,
    Behind,
    Invalid
  };

  /// \brief default Constructor -- does nothing serious
  inline CameraBase()
      : imageWidth_(0),
        imageHeight_(0)
  {
  }

  /// \brief Constructor for width, height and Id
  inline CameraBase(int imageWidth, int imageHeight)
        : imageWidth_(imageWidth),
          imageHeight_(imageHeight)
    {
    }

  /// \brief Destructor -- does nothing
  inline virtual ~CameraBase()
  {
  }

  /// \brief The width of the image in pixels.
  inline uint32_t imageWidth() const
  {
    return imageWidth_;
  }
  /// \brief The height of the image in pixels.
  inline uint32_t imageHeight() const
  {
    return imageHeight_;
  }

  //////////////////////////////////////////////////////////////
  /// \name Methods to project points
  /// @{

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point      The point in Euclidean coordinates.
  /// @param[out] imagePoint The image point.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus project(const Eigen::Vector3d & point,
                                   Eigen::Vector2d * imagePoint) const = 0;

  /// \brief Projects a Euclidean point to a 2d image point (projection).
  ///        Uses projection including distortion models.
  /// @param[in]  point              The point in Euclidean coordinates.
  /// @param[out] imagePoint         The image point.
  /// @param[out] pointJacobian      The Jacobian of the projection function w.r.t. the point..
  /// @param[out] intrinsicsJacobian The Jacobian of the projection function w.r.t. the intinsics.
  /// @return     Get information about the success of the projection. See
  ///             \ref ProjectionStatus for more information.
  virtual ProjectionStatus project(
      const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
      Eigen::Matrix<double, 2, 3> * pointJacobian) const = 0;
			
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to backproject points
  /// @{

  /// \brief Back-project a 2d image point into Euclidean space (direction vector).
  /// @param[in]  imagePoint The image point.
  /// @param[out] direction  The Euclidean direction vector.
  /// @return     true on success.
  virtual bool backProject(const Eigen::Vector2d & imagePoint,
                           Eigen::Vector3d * direction) const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to facilitate unit testing
  /// @{

  /// \brief Creates a random (uniform distribution) image point.
  /// @return A random image point.
  virtual Eigen::Vector2d createRandomImagePoint() const;

  /// \brief Creates a random visible point in Euclidean coordinates.
  /// @param[in] minDist The minimal distance of this point.
  /// @param[in] maxDist The maximum distance of this point.
  /// @return    A random Euclidean point.
  virtual Eigen::Vector3d createRandomVisiblePoint(double minDist = 0.0,
                                                   double maxDist = 10.0) const;
  /// @}

 protected:

  /// \brief Check if the keypoint is in the image.
  inline bool isInImage(const Eigen::Vector2d& imagePoint) const;

  int imageWidth_;  ///< image width in pixels
  int imageHeight_;  ///< image height in pixels
};

}  // namespace cameras
}  // namespace drltools

#include "implementation/CameraBase.hpp"

#endif /* INCLUDE_OKVIS_CAMERAS_CAMERABASE_HPP_ */
