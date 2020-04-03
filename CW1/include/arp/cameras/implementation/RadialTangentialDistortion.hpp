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
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */


#include <Eigen/LU>
#include <iostream>
#include <stdexcept>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion::RadialTangentialDistortion()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0)
{
}

// Constructor initialising ki
RadialTangentialDistortion::RadialTangentialDistortion(double k1, double k2,
                                                       double p1, double p2)
{
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const
{
  double x1 = pointUndistorted(0), x2 = pointUndistorted(1);
  double r2 = pow(x1,2) + pow(x2,2);
  // Calculate coefficient
  double coeff = 1 + k1_ * r2 + k2_ * pow(r2,4);
  // Calculate vector
  Eigen::Vector2d vector;
  vector(0) = 2 * p1_ * x1 * x2 + p2_ * (r2 + 2 * pow(x1,2));
  vector(1) = p1_ * (r2 + 2 * pow(x2,2)) + 2 * p2_ * x1 * x2;
  // Calculate distored point
  *pointDistorted = coeff * pointUndistorted + vector;
  return true;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian) const
{
  bool success = distort(pointUndistorted,pointDistorted);

  // For now: numeric differences on Jacobian.
  // TODO: implement proper Jacobian in practical No. 3
  Eigen::Matrix2d & J = *pointJacobian;
  const double delta = 1.0e-7;
  Eigen::Vector2d pointDistorted_p;
  Eigen::Vector2d pointDistorted_m;
  distort(pointUndistorted+Eigen::Vector2d(delta,0.0), &pointDistorted_p);
  distort(pointUndistorted+Eigen::Vector2d(-delta,0.0), &pointDistorted_m);
  J.col(0) = (pointDistorted_p-pointDistorted_m)*(1.0/(2.0*delta));
  distort(pointUndistorted+Eigen::Vector2d(0.0,delta), &pointDistorted_p);
  distort(pointUndistorted+Eigen::Vector2d(0.0,-delta), &pointDistorted_m);
  J.col(1) = (pointDistorted_p-pointDistorted_m)*(1.0/(2.0*delta));
  return success;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted; // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  if(!success){
    std::cout<<(E.transpose() * E)<<std::endl;
  }

  return success;
}

}  // namespace cameras
}  // namespace arp
