/*
 * VisualInertialTracker.hpp
 *
 *  Created on: 8 Dec 2016
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_VISUALINERTIALTRACKER_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_VISUALINERTIALTRACKER_HPP_

#include <memory>
#include <thread>
#include <mutex>
#include <functional>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include <arp/kinematics/Imu.hpp>
#include <arp/threadsafe/ThreadsafeQueue.hpp>
#include <arp/Frontend.hpp>
#include <arp/ViEkf.hpp>

namespace arp {

///\brief This class synchronises visual and inertial inputs and forwards
///       things to the frontend and estimator.
class VisualInertialTracker
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor -- starts processing thread.
  VisualInertialTracker();

  /// \brief Destructor -- stops threads.
  virtual ~VisualInertialTracker();

  /// \brief Function prototype for output callbacks.
  typedef std::function<
      void(uint64_t timestampMicroseconds, const kinematics::RobotState & state)> EstimatorCallback;

  /// \brief Set the frontend...
  void setFrontend(Frontend& frontend)
  {
    frontend_ = &frontend;
  }

  /// \brief Set the estimator...
  void setEstimator(ViEkf& estimator)
  {
    estimator_ = &estimator;
  }

  /// \brief Set a visualisation callback.
  void setControllerCallback(const EstimatorCallback & controllerCallback) {
    controllerCallback_ = controllerCallback;
  }

  /// \brief Set a visualisation callback.
  void setVisualisationCallback(const EstimatorCallback & visualisationCallback) {
    visualisationCallback_ = visualisationCallback;
  }

  /// \brief Add an image -- non-blocking.
  void addImage(uint64_t timestampMicroseconds, const cv::Mat& image);

  /// \brief Add an IMU measurement -- non-blocking.
  void addImuMeasurement(uint64_t timestampMicroseconds, const Eigen::Vector3d& omega_S,
                         const Eigen::Vector3d& acc_S);

 protected:

  /// \brief This runs in a separate thread, synchronising and then calling
  ///        frontend and estimator functions.
  void processingLoop();

  void controllerLoop();
  void visualisationLoop();

  Frontend* frontend_ = nullptr; ///< The frontend that performs tag detections.
  ViEkf* estimator_ = nullptr; ///< The estimator.

  std::thread processingThread_; ///< Thread consuming asynchronously arriving measurements.
  std::thread controllerThread_; ///< Thread processing results to control something.
  std::thread visualisationThread_; ///< Thread forwarding results for visualisation(s).

  /// \brief Helper struct for IMU measurements.
  /*struct ImuMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t timestampMicroseconds = 0;
    Eigen::Vector3d omega_S;
    Eigen::Vector3d acc_S;
  };*/
  /// \brief Helper struct for images.
  struct CameraMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t timestampMicroseconds = 0;
    cv::Mat image;
  };
  /// \brief Helper struct for state outputs.
  struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t timestampMicroseconds = 0;
    kinematics::RobotState state;
  };

  threadsafe::ThreadSafeQueue<kinematics::ImuMeasurement> imuMeasurementQueue_;
  threadsafe::ThreadSafeQueue<CameraMeasurement> cameraMeasurementQueue_;

  threadsafe::ThreadSafeQueue<StateEstimate> controllerQueue_;
  threadsafe::ThreadSafeQueue<StateEstimate> visualisationQueue_;

  EstimatorCallback controllerCallback_;
  EstimatorCallback visualisationCallback_;

};

}  // namespace arp

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_VISUALINERTIALTRACKER_HPP_ */
