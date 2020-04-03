/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(
      z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  // implementing L4 Slide 30 concerning nonlinear trapezoidal integration
  RobotStateDerivative stateDerivative_k, stateDerivative_k_minus_1;
  continuousTimeNonlinear(state_k_minus_1, z_k_minus_1,
                         stateDerivative_k_minus_1);
  RobotState delta_x_1;
  delta_x_1.r_W = dt * stateDerivative_k_minus_1.r_dot;
  delta_x_1.q_WS.coeffs() = dt * stateDerivative_k_minus_1.q_dot;
  delta_x_1.v_W = dt * stateDerivative_k_minus_1.v_W_dot;
  delta_x_1.b_g = dt * stateDerivative_k_minus_1.b_g_dot;
  delta_x_1.b_a = dt * stateDerivative_k_minus_1.b_a_dot;
  RobotState state_k_tmp;
  state_k_tmp.r_W = state_k_minus_1.r_W + delta_x_1.r_W;
  state_k_tmp.q_WS.coeffs() = (state_k_minus_1.q_WS.coeffs() + delta_x_1.q_WS.coeffs()).normalized();
  state_k_tmp.v_W = state_k_minus_1.v_W + delta_x_1.v_W;
  state_k_tmp.b_g = state_k_minus_1.b_g + delta_x_1.b_g;
  state_k_tmp.b_a = state_k_minus_1.b_a + delta_x_1.b_a;
  continuousTimeNonlinear(state_k_tmp, z_k, stateDerivative_k);
  RobotState delta_x_2;
  delta_x_2.r_W = dt * stateDerivative_k.r_dot;
  delta_x_2.q_WS.coeffs() = dt * stateDerivative_k.q_dot;
  delta_x_2.v_W = dt * stateDerivative_k.v_W_dot;
  delta_x_2.b_g = dt * stateDerivative_k.b_g_dot;
  delta_x_2.b_a = dt * stateDerivative_k.b_a_dot;

  state_k.r_W = state_k_minus_1.r_W + 0.5 * (delta_x_1.r_W + delta_x_2.r_W);
  state_k.q_WS.coeffs() = (state_k_minus_1.q_WS.coeffs()
                           + 0.5 * (delta_x_1.q_WS.coeffs() + delta_x_2.q_WS.coeffs())).normalized();
  state_k.v_W = state_k_minus_1.v_W + 0.5 * (delta_x_1.v_W + delta_x_2.v_W);
  state_k.b_g = state_k_minus_1.b_g + 0.5 * (delta_x_1.b_g + delta_x_2.b_g);
  state_k.b_a = state_k_minus_1.b_a + 0.5 * (delta_x_1.b_a + delta_x_2.b_a);

  if (jacobian) {
    // if requested, impement L4 Slide 30 jacobian of trapezoidal integration with chain rule
    ImuKinematicsJacobian Fc_k_minus_1, Fc_k;
    continuousTimeLinearised(state_k_minus_1, z_k_minus_1, Fc_k_minus_1);
    jacobian->setIdentity();
    *jacobian += 0.5 * dt * Fc_k_minus_1;
    continuousTimeLinearised(state_k_tmp, z_k, Fc_k);
    *jacobian += 0.5 * dt * (Fc_k + dt * Fc_k * Fc_k_minus_1);
  }
  return true;
}

// compute state derivative (v_w_dot = C_WS(a_S + w_a - b_a) + g_W)
bool Imu::continuousTimeNonlinear(const RobotState & state,
                                 const ImuMeasurement & z,
                                 RobotStateDerivative & stateDerivative)
{
  const Eigen::Vector3d omega_S = z.omega_S - state.b_g;
  // implementing L4 slide 28 with no noise
  stateDerivative.r_dot = state.v_W;
  stateDerivative.q_dot = 0.5
      * oplus(Eigen::Quaterniond(0.0, omega_S[0], omega_S[1], omega_S[2]))
      * state.q_WS.coeffs();
  stateDerivative.v_W_dot = state.q_WS.toRotationMatrix() * (z.acc_S - state.b_a)
      + Eigen::Vector3d(0.0, 0.0, -9.81);
  stateDerivative.b_a_dot.setZero();
  stateDerivative.b_g_dot.setZero();
  return true;
}


bool Imu::continuousTimeLinearised(const RobotState & state,
                                  const ImuMeasurement & z,
                                  ImuKinematicsJacobian & jacobian)
{
  // implementing L4 slide 29
  Eigen::Matrix3d C_WS = state.q_WS.toRotationMatrix();
  jacobian.setZero();
  jacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(3, 9) = -C_WS;
  jacobian.block<3, 3>(6, 3) = -crossMx(
      C_WS * (z.acc_S - state.b_a));
  jacobian.block<3, 3>(6, 12) = -C_WS;
  return true;
}

}
}  // namespace arp
