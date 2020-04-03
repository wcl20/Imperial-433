/*
 * ViEkf.cpp
 *
 *  Created on: 20 Nov 2015
 *      Author: sleutene
 */

#include <opencv2/highgui/highgui.hpp>

#include <arp/ViEkf.hpp>
#include <arp/kinematics/Imu.hpp>

namespace arp {

ViEkf::ViEkf()
    : cameraModel_(0, 0, 0, 0, 0, 0, arp::cameras::NoDistortion())
{
  x_.r_W.setZero();
  x_.q_WS.setIdentity();
  x_.v_W.setZero();
  x_.b_g.setZero();
  x_.b_a.setZero();
  P_.setZero();
}

// Add a marker target.
bool ViEkf::setTarget(unsigned int j,
                      const arp::kinematics::Transformation & T_WTj,
                      double targetSizeMetres)
{
  T_WT_[j] = T_WTj;
  targetSizesMetres_[j] = targetSizeMetres;
  return true;
}

// Remove a marker target.
bool ViEkf::removeTarget(unsigned int j)
{
  return 2 == T_WT_.erase(j) + targetSizesMetres_.erase(j);  // both have to erase one element...
}

// Set the pose of the camera relative to the IMU.
bool ViEkf::setCameraExtrinsics(const arp::kinematics::Transformation & T_SC)
{
  T_SC_ = T_SC;
  return true;
}

// Set the intrinsics of the camera, i.e. the camera model
bool ViEkf::setCameraIntrinsics(
    const arp::cameras::PinholeCamera<arp::cameras::NoDistortion> & cameraModel)
{
  cameraModel_ = cameraModel;
  return true;
}

// Set IMU noise parameters
void ViEkf::setImuNoiseParameters(double sigma_c_gyr, double sigma_c_acc,
                                  double sigma_c_gw, double sigma_c_aw)
{
  sigma_c_gyr_ = sigma_c_gyr;
  sigma_c_acc_ = sigma_c_acc;
  sigma_c_gw_ = sigma_c_gw;
  sigma_c_aw_ = sigma_c_aw;
}

// Set measurement noise parameter.
void ViEkf::setDetectorNoiseParameter(double sigma_imagePoint)
{
  sigma_imagePoint_ = sigma_imagePoint;
}

// Initialise the states
bool ViEkf::initialiseState(uint64_t timestampMicroseconds,
                            const arp::kinematics::RobotState & x,
                            const Eigen::Matrix<double, 15, 15> & P)
{
  timestampLastUpdateMicrosec_ = timestampMicroseconds;
  x_ = x;
  P_ = P;

  // reset propagation for publishing
  x_propagated_ = x_;
  timestampPropagatedMicrosec_ = timestampLastUpdateMicrosec_;

  return true;
}

// Has this EKF been initialised?
bool ViEkf::isInitialised() const
{
  return 0 != timestampLastUpdateMicrosec_;
}

// Get the states.
bool ViEkf::getState(uint64_t timestampMicroseconds,
                     arp::kinematics::RobotState & x,
                     Eigen::Matrix<double, 15, 15> * P)
{
  if (timestampPropagatedMicrosec_ == 0) {
    x = x_;
    if (P) {
      *P = P_;
    }
    return false;
  }

  if (timestampPropagatedMicrosec_ > timestampLastUpdateMicrosec_) {
    if (timestampPropagatedMicrosec_ - timestampLastUpdateMicrosec_ > 100000) {
      // stop propagation, this will just diverge
      // assign output
      x = x_propagated_;
      if (P) {
        *P = P_;  // Not 100% correct, we should have also propagated  P_...
      }
      return false;
    }
  }

  // run prediction as far as possible
  for (auto it_k_minus_1 = imuMeasurementDeque_.begin();
      it_k_minus_1 != imuMeasurementDeque_.end(); ++it_k_minus_1) {
    auto it_k = it_k_minus_1;
    it_k++;
    if (it_k == imuMeasurementDeque_.end()) {
      return false;  // we reached the buffer end...
    }

    // ensure we're in the right segment
    if (it_k->timestampMicroseconds < timestampPropagatedMicrosec_) {
      continue;
    }

    if (it_k->timestampMicroseconds >= timestampMicroseconds) {
      break;
    }

    // propagate and get state transition matrix
    arp::kinematics::ImuKinematicsJacobian F =
        arp::kinematics::ImuKinematicsJacobian::Identity();
    arp::kinematics::RobotState x_start = x_propagated_;
    arp::kinematics::Imu::stateTransition(x_start, *it_k_minus_1, *it_k,
                                          x_propagated_, &F);
  }
  // assign output
  x = x_propagated_;
  if (P) {
    *P = P_;  // Not 100% correct, we should have also propagated  P_...
  }

  // remember
  timestampPropagatedMicrosec_ = timestampMicroseconds;
  return true;
}

// Obtain the target pose of target id.
bool ViEkf::get_T_WT(int id, kinematics::Transformation & T_WT) const
{
  // check if target was registered:
  if (targetSizesMetres_.find(id) == targetSizesMetres_.end()) {
    return false;
  }
  T_WT = T_WT_.at(id);
  return true;
}

bool ViEkf::addImuMeasurement(uint64_t timestampMicroseconds,
                              const Eigen::Vector3d & gyroscopeMeasurements,
                              const Eigen::Vector3d & accelerometerMeasurements)
{
  imuMeasurementDeque_.push_back(
      arp::kinematics::ImuMeasurement { timestampMicroseconds,
          gyroscopeMeasurements, accelerometerMeasurements });
  return true;
}

// The EKF prediction.
bool ViEkf::predict(uint64_t from_timestampMicroseconds,
                    uint64_t to_timestampMicroseconds)
{
  for (auto it_k_minus_1 = imuMeasurementDeque_.begin();
      it_k_minus_1 != imuMeasurementDeque_.end(); ++it_k_minus_1) {
    auto it_k = it_k_minus_1;
    it_k++;
    if (it_k == imuMeasurementDeque_.end()) {
      break;  // we reached the buffer end...
    }

    // ensure we're in the right segment
    if (it_k->timestampMicroseconds < from_timestampMicroseconds
        || it_k->timestampMicroseconds >= to_timestampMicroseconds) {
      continue;
    }

    // access the IMU measurements:
    kinematics::ImuMeasurement z_k_minus_1 = *it_k_minus_1; // IMU meas. at last time step
    kinematics::ImuMeasurement z_k = *it_k; // IMU measurements at current time step

    // get the time delta
    const double delta_t = double(z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;

    // TODO: propagate robot state x_ using IMU measurements
    // i.e. we do x_k = f(x_k_minus_1).
    // Also, we compute the matrix F (linearisation of f()) related to
    // delta_chi_k = F * delta_chi_k_minus_1.
    kinematics::ImuKinematicsJacobian F_k;
    kinematics::Imu::stateTransition(x_, z_k_minus_1, z_k, x_, &F_k);

    // TODO: propagate covariance matrix P_
    // Compute noise matrix LQL^T
    kinematics::ImuKinematicsJacobian LQL;
    LQL.setZero();
    LQL.block<3,3>(3,3) = pow(sigma_c_gw_, 2) * delta_t * Eigen::Matrix3d::Identity();
    LQL.block<3,3>(6,6) = pow(sigma_c_aw_, 2) * delta_t * Eigen::Matrix3d::Identity();
    LQL.block<3,3>(9,9) = pow(sigma_c_gyr_, 2) * delta_t * Eigen::Matrix3d::Identity();
    LQL.block<3,3>(12,12) = pow(sigma_c_acc_, 2) * delta_t * Eigen::Matrix3d::Identity();
    P_ = F_k * P_ * F_k.transpose() + LQL;
  }
  return true;  // TODO: change to true once implemented
}

// Pass a set of corner measurements to trigger an update.
bool ViEkf::addTargetMeasurement(
    uint64_t timestampMicroseconds,
    const Eigen::Matrix<double, 2, 4> & imagePointMeasurements, int id)
{
  // let's do the propagation from last time to now:
  predict(timestampLastUpdateMicrosec_, timestampMicroseconds);

  // check if target was registered:
  if (targetSizesMetres_.find(id) == targetSizesMetres_.end()) {
    return false;
  }

  // now we are ready to do the actual update
  bool success = true;
  for (size_t k = 0; k < 4; ++k) {
    success &= update(timestampMicroseconds, imagePointMeasurements, id, k);
  }

  // monitor update history
  if (!success) {
    lastTimeReject_ = true;
    if (lastTimeReject_) {
      rejections_++;
    }
  } else {
    // reset monitoring
    lastTimeReject_ = false;
    rejections_ = 0;
  }

  // remember update
  timestampLastUpdateMicrosec_ = timestampMicroseconds;
  auto it = imuMeasurementDeque_.begin();

  // also delete stuff in the queue since not needed anymore.
  while (it != imuMeasurementDeque_.end()) {
    auto it_p1 = it;
    it_p1++;
    if (it_p1 == imuMeasurementDeque_.end()) {
      break;
    }
    if (it->timestampMicroseconds < timestampMicroseconds
        && it_p1->timestampMicroseconds >= timestampMicroseconds) {
      break;
    }
    it++;
  }
  imuMeasurementDeque_.erase(imuMeasurementDeque_.begin(), it);

  // reset propagation for publishing
  x_propagated_ = x_;
  timestampPropagatedMicrosec_ = timestampLastUpdateMicrosec_;

  if (rejections_ > 3) {
    std::cout << "REINITIALISE" << std::endl;
    timestampLastUpdateMicrosec_ = 0;  // not initialised.
    rejections_ = 0;
    lastTimeReject_ = false;
  }
  return true;
}

// The EKF update.
bool ViEkf::update(uint64_t timestampMicroseconds,
                   const Eigen::Matrix<double, 2, 4> & imagePointMeasurements,
                   int id, unsigned int p)
{

  // imagePointMeasurements are the corners of the tag we found in the
  // image (undistorted, in pixels [u,v]). We are requested to do an
  // update using the p-th corner (i.e. one at the time).

  // pose T_WS: transforms points represented in Sensor(IMU) coordinates
  // into World coordinates
  kinematics::Transformation T_WS(x_.r_W, x_.q_WS);

  // the point in world coordinates
  const double ds = targetSizesMetres_[id] * 0.5;

  // These are the corners of the AprilTag represented
  // in the "tag" frame T
  Eigen::Matrix4d allPoints;
  allPoints << -ds,  ds,  ds, -ds,
               -ds, -ds,  ds,  ds,
               0.0, 0.0, 0.0, 0.0,
               1.0, 1.0, 1.0, 1.0;

  // get the right point from tag and transform to World frame
  // select the point (corner index p) for which we do the update:
  const Eigen::Vector4d hp_T = allPoints.col(p);

  // let's find the pose of the AprilTag (must have been registered):
  if(T_WT_.find(id) == T_WT_.end()){
    throw std::runtime_error("id not registered");
  }
  // let's get that transformation using this tag's (unique!) id
  // and transform the corner point into the World frame:
  const Eigen::Vector4d hp_W = T_WT_[id] * hp_T;

  // TODO: transform the corner point from world frame into the camera frame
  // (remember the camera projection will assume the point is represented
  // in camera coordinates):
  const Eigen::Vector4d hp_C = T_SC().inverse() * T_WS.inverse() * hp_W;

  // TODO: calculate the reprojection error y (residual)
  // using the PinholeCamera::project
  Eigen::Vector2d y;
  // Transform point from homogeneous point to normal 3D point
  const Eigen::Vector3d hp_C_3d = 1 / hp_C(3) * hp_C.head(3);
  // Compute projected point h(x)
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> U; // Jacobian matrix U used for next step
  cameras::CameraBase::ProjectionStatus status = cameraModel_.project(hp_C_3d, &imagePoint, &U);
  // Compute Residual
  y = imagePointMeasurements.col(p) - imagePoint;

  // TODO: check validity of projection -- return false if not successful!
  if (status != cameras::CameraBase::ProjectionStatus::Successful) {
    return false;
  }

  // TODO: calculate measurement Jacobian H
  Eigen::Matrix3d C_CS = T_SC().inverse().C();
  Eigen::Matrix3d C_WS = x_.q_WS.toRotationMatrix();
  Eigen::Matrix<double, 2, 15> H;
  H.setZero();
  H.block<2, 3>(0, 0) = U * C_CS * -C_WS.transpose();
  // Transform point from homogeneous point to normal 3D point
  const Eigen::Vector3d hp_W_3d = 1 / hp_W(3) * hp_W.head(3);
  H.block<2, 3>(0, 3) = U * C_CS * C_WS.transpose() * kinematics::crossMx(hp_W_3d - x_.r_W);

  // Obtain the measurement covariance form parameters:
  const double r = sigma_imagePoint_ * sigma_imagePoint_;
  Eigen::Matrix2d R = Eigen::Vector2d(r, r).asDiagonal();  // the measurement covariance

  // TODO: compute residual covariance S
  Eigen::Matrix2d S = H * P_ * H.transpose() + R;

  // chi2 test -- See Lecture 4, Slide 21
  if (y.transpose() * S.inverse() * y > 40.0) {
    // std::cout << "Rejecting measurement, residual = " << y.transpose()
        // << " pixels" << std::endl;
    return false;
  }

  // TODO: compute Kalman gain K
  Eigen::Matrix<double, 15, 2> K = P_ * H.transpose() * S.inverse();

  // TODO: compute increment Delta_chi
  Eigen::VectorXd delta_chi = K * y;

  // TODO: perform update. Note: multiplicative for the quaternion!!
  x_.r_W += delta_chi.segment<3>(0);
  x_.q_WS = kinematics::oplus(x_.q_WS) * arp::kinematics::deltaQ(delta_chi.segment<3>(3)).coeffs();
  x_.q_WS.normalize();
  x_.v_W += delta_chi.segment<3>(6);
  x_.b_g += delta_chi.segment<3>(9);
  x_.b_a += delta_chi.segment<3>(12);

  // TODO: update to covariance matrix:
  P_ = P_ - K * H * P_;

  return true;  // TODO: change to true once implemented...
}

}  // namespace arp
