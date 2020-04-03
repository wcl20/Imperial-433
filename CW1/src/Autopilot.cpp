/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>
#include <math.h>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode

  arp::PidController::Parameters controllerParameters;

  controllerParameters.k_p = 0.05;
  controllerParameters.k_i = 0.00;
  controllerParameters.k_d = 0.05;
  x_pid.setParameters(controllerParameters);
  y_pid.setParameters(controllerParameters);

  controllerParameters.k_p = 1.0;
  controllerParameters.k_i = 0.0;
  controllerParameters.k_d = 0.0;
  z_pid.setParameters(controllerParameters);

  controllerParameters.k_p = 1.5;
  controllerParameters.k_i = 0.0;
  controllerParameters.k_d = 0.0;
  yaw_pid.setParameters(controllerParameters);

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone battery status.
float Autopilot::droneBattery()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return navdata.batteryPercent;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  if(navdata.altd < 600.0 && (navdata.state == arp::Autopilot::Flying ||
                              navdata.state == arp::Autopilot::Hovering ||
                              navdata.state == arp::Autopilot::Flying2)) {
    // so, effectively the ardrone_autonomy package is doing something weird. Let's fix.
    // \todo The 600 mm threshold is quite arbitrary -- this should not be hard-coded.
    navdata.state = arp::Autopilot::Unknown;
  }
  return DroneStatus(navdata.state);
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  if(!isAutomatic_) {
    return move(forward, left, up, rotateLeft);
  }
  return false;
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  // process moving commands when in state 3,4, or 7
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Flying &&
    status != DroneStatus::Hovering &&
    status != DroneStatus::Flying2) {
    return false;
  }
  geometry_msgs::Twist moveMsg;
  moveMsg.linear.x = forward;
  moveMsg.linear.y = left;
  moveMsg.linear.z = up;
  moveMsg.angular.z = rotateLeft;
  pubMove_.publish(moveMsg);

  return true;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{
  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.r_W[0], x.r_W[1], x.r_W[2], yaw);
    return;
  }

  // TODO: only enable when in flight

  Eigen::Vector3d positionReference;
  double yaw_ref;

  // Get waypoint list, if available
  std::lock_guard<std::mutex> l(waypointMutex_);
  if(!waypoints_.empty()) {
    // TODO: setPoseReference() from current waypoint
    auto waypoint = waypoints_.front();
    setPoseReference(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw);

    getPoseReference(positionReference[0], positionReference[1],
      positionReference[2], yaw_ref);
    // TODO: remove the current waypoint, if the position error is below the tolerance.
    Eigen::Matrix3d C_SW = x.q_WS.inverse().toRotationMatrix();
    Eigen::Vector3d err_r = C_SW * (positionReference - x.r_W);

    double distance = err_r.norm();
    std::cout << "distance from point: " << distance << std::endl;
    if (distance < waypoint.posTolerance) {
      waypoints_.pop_front();
    }
  } else {
    // This is the original line of code:
    getPoseReference(positionReference[0], positionReference[1],
      positionReference[2], yaw_ref);
  }

  // Compute error signals
  Eigen::Matrix3d C_SW = x.q_WS.inverse().toRotationMatrix();
  Eigen::Vector3d err_r = C_SW * (positionReference - x.r_W);

  double err_yaw = yaw_ref - arp::kinematics::yawAngle(x.q_WS);

  // Ensure err_yaw is within the limits of [-pi, pi]
  if (err_yaw < -M_PI) {
    err_yaw += 2 * M_PI;
  } else if (err_yaw > M_PI) {
    err_yaw -= 2 * M_PI;
  }

  // Compute error signal time derivatives
  Eigen::Vector3d err_r_dot = - C_SW * x.v_W;
  double err_yaw_dot = 0;

  // Get PID controller outputs (ROS parameters)
  double euler_angle_max, control_vz_max, control_yaw;
  nh_->getParam("/ardrone_driver/euler_angle_max", euler_angle_max);
  nh_->getParam("/ardrone_driver/control_vz_max", control_vz_max);
  nh_->getParam("/ardrone_driver/control_yaw", control_yaw);

  x_pid.setOutputLimits(-euler_angle_max, euler_angle_max);
  y_pid.setOutputLimits(-euler_angle_max, euler_angle_max);
  z_pid.setOutputLimits(-control_vz_max * 1e-3, control_vz_max * 1e-3);
  yaw_pid.setOutputLimits(-control_yaw, control_yaw);

  // Compute control output
  double x_output = x_pid.control(timeMicroseconds, err_r(0), err_r_dot(0));
  double y_output = y_pid.control(timeMicroseconds, err_r(1), err_r_dot(1));
  double z_output = z_pid.control(timeMicroseconds, err_r(2), err_r_dot(2));
  double yaw_output = yaw_pid.control(timeMicroseconds, err_yaw, err_yaw_dot);

  // Regularise values before sending to move, which has limit [-1, 1]
  x_output /= euler_angle_max;
  y_output /= euler_angle_max;
  z_output /= control_vz_max * 1e-3;
  yaw_output /= control_yaw;

  // Send to move
  move(x_output, y_output, z_output, yaw_output);
}

}  // namespace arp
