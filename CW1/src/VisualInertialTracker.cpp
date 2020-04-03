/*
 * VisualInertialTracker.cpp
 *
 *  Created on: 8 Dec 2016
 *      Author: sleutene
 */

#include <arp/VisualInertialTracker.hpp>

namespace arp {

VisualInertialTracker::VisualInertialTracker()
{
  processingThread_ = std::thread(&VisualInertialTracker::processingLoop, this);
  controllerThread_ = std::thread(&VisualInertialTracker::controllerLoop, this);
  visualisationThread_ = std::thread(&VisualInertialTracker::visualisationLoop, this);
}

VisualInertialTracker::~VisualInertialTracker()
{
  imuMeasurementQueue_.Shutdown();
  cameraMeasurementQueue_.Shutdown();
  controllerQueue_.Shutdown();
  visualisationQueue_.Shutdown();
  processingThread_.join();
  controllerThread_.join();
  visualisationThread_.join();
}

void VisualInertialTracker::addImage(uint64_t timestampMicroseconds,
                                     const cv::Mat& image)
{
  CameraMeasurement cameraMeasurement;
  cameraMeasurement.timestampMicroseconds = timestampMicroseconds;
  cameraMeasurement.image = image;
  cameraMeasurementQueue_.PushNonBlockingDroppingIfFull(cameraMeasurement,2);
}

void VisualInertialTracker::addImuMeasurement(uint64_t timestampMicroseconds,
                                              const Eigen::Vector3d& omega_S,
                                              const Eigen::Vector3d& acc_S)
{
  kinematics::ImuMeasurement imuMeasurement;
  imuMeasurement.timestampMicroseconds = timestampMicroseconds;
  imuMeasurement.omega_S = omega_S;
  imuMeasurement.acc_S = acc_S;
  imuMeasurementQueue_.PushNonBlocking(imuMeasurement);
}

void VisualInertialTracker::processingLoop()
{
  // never end
  for (;;) {

    // get camera measurement
    CameraMeasurement cameraMeasurement;
    if (!cameraMeasurementQueue_.PopBlocking(&cameraMeasurement)) {
      return;
    } else {

      // get IMU measurement
      kinematics::ImuMeasurement imuMeasurement;
      uint64_t t = 0;

      while (t<cameraMeasurement.timestampMicroseconds) {
        if(!imuMeasurementQueue_.PopBlocking(&imuMeasurement)){
          return;
        }
        // feed to estimator (propagation)
        estimator_->addImuMeasurement(imuMeasurement.timestampMicroseconds,
                                           imuMeasurement.omega_S,
                                           imuMeasurement.acc_S);
        // publish to controller (high rate)
        kinematics::RobotState x;
        t = imuMeasurement.timestampMicroseconds;
        if (estimator_->getState(t, x)) {
          if(controllerCallback_) {
            StateEstimate estimate;
            estimate.timestampMicroseconds = t;
            estimate.state = x;
            controllerQueue_.PushNonBlocking(estimate);
          }
        }
      }

      if (cameraMeasurement.timestampMicroseconds > t)
        std::cout << "BAD" << std::endl;
      Frontend::DetectionVec detections;
      bool success = frontend_->detect(cameraMeasurement.image, detections);
      if (success) {

        // feed to estimator (measurement update)
        if (estimator_->isInitialised()) {
          for (size_t i = 0; i < detections.size(); ++i) {
            estimator_->addTargetMeasurement(
                cameraMeasurement.timestampMicroseconds, detections[i].points,
                detections[i].id);
          }
        } else {
          // find best target to initialise
          double dist = 1000.0;  //m
          int bestI = 0;
          for (size_t i = 0; i < detections.size(); ++i) {
            const double thisDist = detections[i].T_CT.r().norm();
            if (thisDist < dist) {
              dist = thisDist;
              bestI = i;
            }
          }
          // init state and covar
          kinematics::RobotState x;
          Eigen::Matrix<double, 15, 15> P =
              Eigen::Matrix<double, 15, 15>::Identity();
          kinematics::Transformation T_WT;
          estimator_->get_T_WT(detections[bestI].id, T_WT);
          kinematics::Transformation T_WS = (T_WT
              * detections[bestI].T_CT.inverse() * estimator_->T_SC().inverse());
          x.r_W = T_WS.r();
          x.q_WS = T_WS.q();
          x.v_W.setZero();
          x.b_g.setZero();
          x.b_a.setZero();
          P.block<3, 3>(0, 0) *= 0.0001 * bestI * bestI;  // 1 cm
          P.block<3, 3>(3, 3) *= (0.5 / 60.0) * (0.5 / 60.0);  // 0.5 degree
          P.block<3, 3>(6, 6) *= 0.01 * 0.01;  // 10 cm/s
          P.block<3, 3>(9, 9) *= (1.0 / 60.0) * (1.0 / 60.0);  // 1 degree/sec
          P.block<3, 3>(12, 12) *= 0.1;  // 1 m/s^2
          estimator_->initialiseState(cameraMeasurement.timestampMicroseconds,
                                      x, P);
        }
      }

      // publish for visualisation
      kinematics::RobotState x;
      if (estimator_->getState(cameraMeasurement.timestampMicroseconds, x)) {
        if(visualisationCallback_) {
          StateEstimate estimate;
          estimate.timestampMicroseconds = cameraMeasurement.timestampMicroseconds;
          estimate.state = x;
          visualisationQueue_.PushNonBlocking(estimate);
        }
      }
    }
  }
}

void VisualInertialTracker::controllerLoop()
{
  // never end
  for (;;) {
    // get estimator output
    StateEstimate stateEstimate;
    if (!controllerQueue_.PopBlocking(&stateEstimate)) {
      return;
    } else {
      if (controllerCallback_) {
        controllerCallback_(stateEstimate.timestampMicroseconds, stateEstimate.state);
      }
    }
  }
}

void VisualInertialTracker::visualisationLoop()
{
  // never end
  for (;;) {
    // get estimator output
    StateEstimate stateEstimate;
        if (!visualisationQueue_.PopBlocking(&stateEstimate)) {
      return;
    } else {
      if (visualisationCallback_) {
        visualisationCallback_(stateEstimate.timestampMicroseconds, stateEstimate.state);
      }
    }
  }
}

}  // namespace arp

