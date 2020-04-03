// Bring in my package's API, which is what I'm testing
#include "arp/ViEkf.hpp"
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
#include "arp/kinematics/Imu.hpp"
#include "arp/kinematics/Transformation.hpp"

// Bring in gtest
#include <gtest/gtest.h>

#include <cmath>

#include <atomic>
#include <iostream>

namespace { // anonymous namespace to protect local definitions (e.g. ImuTest)

// We require a derived class, since the to-be-tested functions are declared protected and
// otherwise not accessible.
class ImuTest final : public arp::kinematics::Imu {
  public:
   typedef arp::kinematics::Imu Base;
   using Base::continuousTimeNonlinear;
   using Base::continuousTimeLinearised;
};

// We require a derived class, since the to-be-tested functions are declared protected and
// otherwise not accessible.
class ViEkfTest final : public arp::ViEkf {
  public:
   typedef arp::ViEkf Base;
   using Base::predict;
   using Base::update;
};

TEST(ImuKinematics, numericDifferencesDiscreteTime)
{
  // generate random state
  arp::kinematics::RobotState state;
  state.r_W.setRandom();
  state.q_WS.coeffs() = Eigen::Vector4d::Random();
  state.v_W.setRandom();
  state.b_g.setRandom();
  state.b_a.setRandom();
  state.q_WS.normalize();

  // random measurement
  arp::kinematics::ImuMeasurement measurement_0, measurement_1;
  measurement_0.timestampMicroseconds = 0;
  measurement_0.omega_S.setRandom();
  measurement_0.acc_S.setRandom();
  measurement_1.timestampMicroseconds = 10000;
  measurement_1.omega_S = measurement_0.omega_S + Eigen::Vector3d::Random()*0.01;
  measurement_1.acc_S = measurement_0.acc_S + Eigen::Vector3d::Random()*0.01;

  // evaluate analytic Jacobian:
  arp::kinematics::ImuKinematicsJacobian F;
  arp::kinematics::RobotState state_1;
  ImuTest::stateTransition(state, measurement_0, measurement_1, state_1, &F);
  
  // num diff Jacobian (central differences)
  const double delta = 1.0e-5;
  arp::kinematics::ImuKinematicsJacobian F_numDiff;
  Eigen::Quaterniond q_SW = state_1.q_WS.inverse();
  for(size_t i=0; i<3; ++i) {
    arp::kinematics::RobotState x_1_p, x_1_m;
    arp::kinematics::RobotState x_0_p = state;
    x_0_p.r_W[i] += delta;
    ImuTest::stateTransition(x_0_p, measurement_0, measurement_1, x_1_p);
    arp::kinematics::RobotState x_0_m = state;
    x_0_m.r_W[i] -= delta;
    ImuTest::stateTransition(x_0_m, measurement_0, measurement_1, x_1_m);
    Eigen::Matrix<double,16,1> delta_x;
    delta_x.head<3>() = (x_1_p.r_W-x_1_m.r_W)/(2.0*delta);
    delta_x.segment<4>(3) = (x_1_p.q_WS.coeffs()-x_1_m.q_WS.coeffs())/(2.0*delta);
    delta_x.segment<3>(7) = (x_1_p.v_W-x_1_m.v_W)/(2.0*delta);
    delta_x.segment<3>(10) = (x_1_p.b_g-x_1_m.b_g)/(2.0*delta);
    delta_x.tail<3>() = (x_1_p.b_a-x_1_m.b_a)/(2.0*delta);

    // now back to mimimal dimensions
    F_numDiff.block<3,1>(0,i) = delta_x.segment<3>(0);
    F_numDiff.block<3,1>(3,i) = 2*(arp::kinematics::oplus(q_SW)*delta_x.segment<4>(3)).head<3>();
    F_numDiff.block<9,1>(6,i) = delta_x.segment<9>(7);
  }
  for(size_t i=0; i<3; ++i) {
    arp::kinematics::RobotState x_1_p, x_1_m;
    arp::kinematics::RobotState x_0_p = state;
    Eigen::Vector3d deltaAlpha = Eigen::Vector3d::Zero();
    deltaAlpha[i] += delta;
    x_0_p.q_WS = (arp::kinematics::deltaQ(deltaAlpha) * x_0_p.q_WS);
    ImuTest::stateTransition(x_0_p, measurement_0, measurement_1, x_1_p);
    arp::kinematics::RobotState x_0_m = state;
    x_0_m.q_WS = (arp::kinematics::deltaQ(-deltaAlpha) * x_0_m.q_WS);
    ImuTest::stateTransition(x_0_m, measurement_0, measurement_1, x_1_m);
    Eigen::Matrix<double,16,1> delta_x;
    delta_x.head<3>() = (x_1_p.r_W-x_1_m.r_W)/(2.0*delta);
    delta_x.segment<4>(3) = (x_1_p.q_WS.coeffs()-x_1_m.q_WS.coeffs())/(2.0*delta);
    delta_x.segment<3>(7) = (x_1_p.v_W-x_1_m.v_W)/(2.0*delta);
    delta_x.segment<3>(10) = (x_1_p.b_g-x_1_m.b_g)/(2.0*delta);
    delta_x.tail<3>() = (x_1_p.b_a-x_1_m.b_a)/(2.0*delta);

    // now back to mimimal dimensions
    F_numDiff.block<3,1>(0,i+3) = delta_x.segment<3>(0);
    F_numDiff.block<3,1>(3,i+3) = 2*(arp::kinematics::oplus(q_SW)*delta_x.segment<4>(3)).head<3>();
    F_numDiff.block<9,1>(6,i+3) = delta_x.segment<9>(7);
  }
  for(size_t i=0; i<3; ++i) {
    arp::kinematics::RobotState x_1_p, x_1_m;
    arp::kinematics::RobotState x_0_p = state;
    x_0_p.v_W[i] += delta;
    ImuTest::stateTransition(x_0_p, measurement_0, measurement_1, x_1_p);
    arp::kinematics::RobotState x_0_m = state;
    x_0_m.v_W[i] -= delta;
    ImuTest::stateTransition(x_0_m, measurement_0, measurement_1, x_1_m);
    Eigen::Matrix<double,16,1> delta_x;
    delta_x.head<3>() = (x_1_p.r_W-x_1_m.r_W)/(2.0*delta);
    delta_x.segment<4>(3) = (x_1_p.q_WS.coeffs()-x_1_m.q_WS.coeffs())/(2.0*delta);
    delta_x.segment<3>(7) = (x_1_p.v_W-x_1_m.v_W)/(2.0*delta);
    delta_x.segment<3>(10) = (x_1_p.b_g-x_1_m.b_g)/(2.0*delta);
    delta_x.tail<3>() = (x_1_p.b_a-x_1_m.b_a)/(2.0*delta);

    // now back to mimimal dimensions
    F_numDiff.block<3,1>(0,i+6) = delta_x.segment<3>(0);
    F_numDiff.block<3,1>(3,i+6) = 2*(arp::kinematics::oplus(q_SW)*delta_x.segment<4>(3)).head<3>();
    F_numDiff.block<9,1>(6,i+6) = delta_x.segment<9>(7);
  }
  for(size_t i=0; i<3; ++i) {
    arp::kinematics::RobotState x_1_p, x_1_m;
    arp::kinematics::RobotState x_0_p = state;
    x_0_p.b_g[i] += delta;
    ImuTest::stateTransition(x_0_p, measurement_0, measurement_1, x_1_p);
    arp::kinematics::RobotState x_0_m = state;
    x_0_m.b_g[i] -= delta;
    ImuTest::stateTransition(x_0_m, measurement_0, measurement_1, x_1_m);
    Eigen::Matrix<double,16,1> delta_x;
    delta_x.head<3>() = (x_1_p.r_W-x_1_m.r_W)/(2.0*delta);
    delta_x.segment<4>(3) = (x_1_p.q_WS.coeffs()-x_1_m.q_WS.coeffs())/(2.0*delta);
    delta_x.segment<3>(7) = (x_1_p.v_W-x_1_m.v_W)/(2.0*delta);
    delta_x.segment<3>(10) = (x_1_p.b_g-x_1_m.b_g)/(2.0*delta);
    delta_x.tail<3>() = (x_1_p.b_a-x_1_m.b_a)/(2.0*delta);

    // now back to mimimal dimensions
    F_numDiff.block<3,1>(0,i+9) = delta_x.segment<3>(0);
    F_numDiff.block<3,1>(3,i+9) = 2*(arp::kinematics::oplus(q_SW)*delta_x.segment<4>(3)).head<3>();
    F_numDiff.block<9,1>(6,i+9) = delta_x.segment<9>(7);
  }
  for(size_t i=0; i<3; ++i) {
    arp::kinematics::RobotState x_1_p, x_1_m;
    arp::kinematics::RobotState x_0_p = state;
    x_0_p.b_a[i] += delta;
    ImuTest::stateTransition(x_0_p, measurement_0, measurement_1, x_1_p);
    arp::kinematics::RobotState x_0_m = state;
    x_0_m.b_a[i] -= delta;
    ImuTest::stateTransition(x_0_m, measurement_0, measurement_1, x_1_m);
    Eigen::Matrix<double,16,1> delta_x;
    delta_x.head<3>() = (x_1_p.r_W-x_1_m.r_W)/(2.0*delta);
    delta_x.segment<4>(3) = (x_1_p.q_WS.coeffs()-x_1_m.q_WS.coeffs())/(2.0*delta);
    delta_x.segment<3>(7) = (x_1_p.v_W-x_1_m.v_W)/(2.0*delta);
    delta_x.segment<3>(10) = (x_1_p.b_g-x_1_m.b_g)/(2.0*delta);
    delta_x.tail<3>() = (x_1_p.b_a-x_1_m.b_a)/(2.0*delta);

    // now back to mimimal dimensions
    F_numDiff.block<3,1>(0,i+12) = delta_x.segment<3>(0);
    F_numDiff.block<3,1>(3,i+12) = 2*(arp::kinematics::oplus(q_SW)*delta_x.segment<4>(3)).head<3>();
    F_numDiff.block<9,1>(6,i+12) = delta_x.segment<9>(7);
  }
  //std::cout << "F_numDiff=\n" << F_numDiff << std::endl;
  //std::cout << "F=\n" << F << std::endl;
  if((F-F_numDiff).norm()>=1.0e-5)
    std::cout << "Wrong Jacobian: F - F_numDiff = " << std::endl << (F-F_numDiff) << std::endl;
  // now num-diff should match analytical
  EXPECT_TRUE((F-F_numDiff).norm()<1.0e-5);
}

TEST(ViEkfTest, predictState) {

  // construct updateable ekf
  ViEkfTest viEkfTest;

  // arbitrary extrinsics
  arp::kinematics::Transformation T_SC(Eigen::Vector3d(1,2,3),Eigen::Quaterniond(1,2,3,4).normalized());
  viEkfTest.setCameraExtrinsics(T_SC);

  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::NoDistortion>::testObject();
  viEkfTest.setCameraIntrinsics(pinholeCamera);

  // initialise random state
  arp::kinematics::RobotState state;
  state.r_W << 0.902131,   1.60338,   2.64148;
  state.q_WS.coeffs() << 5.2336,  -2.66092,  0.702473,  -2.00367;
  state.q_WS.normalize();
  state.v_W << -2.29765,  0.704136,   1.51915;
  state.b_g << 3.82437,  0.499508,   2.02314;
  state.b_a << -1.28832,   1.77941,  0.307377;
  Eigen::Matrix<double, 15, 15> P;
  P <<   
  6.84348,  0.317284,   1.42921,  0.902131,  -1.09333, -0.254766,   -3.2244,   2.61621,   2.27235,   1.10791, -0.637078,  0.328422,  0.527314,   1.03198,  0.307377,
 0.317284,   3.58218,   1.53043,   1.60338, -0.287659, -0.337079,  -1.24307, -0.442813,   1.48224, -0.463591,   1.19463, -0.623509,   0.13328, -0.954335, -0.135775,
  1.42921,   1.53043,   4.36309,   2.64148, -0.485696,  0.706116,  -1.17031,  0.421821,   1.83664,  0.745006,   1.05798, -0.867287,   1.11493,  0.221352, -0.192116,
 0.902131,   1.60338,   2.64148,    5.2336,  -2.66092,  0.702473,  -2.00367,  -2.29765,  0.704136,   1.51915,   3.82437,  0.499508,   2.02314,  -1.28832,   1.77941,
 -1.09333, -0.287659, -0.485696,  -2.66092,    4.9174,  0.450124, -0.525324,  0.508448,  0.955693,  -1.79853,  -1.60548,  -0.66799, -0.259653, -0.860349,    1.0568,
-0.254766, -0.337079,  0.706116,  0.702473,  0.450124,   4.30821,  -1.07022,  -4.35496,  0.785995,  -1.06488,   3.11436,  -1.53233,   2.47007,  -1.65243,   1.61018,
  -3.2244,  -1.24307,  -1.17031,  -2.00367, -0.525324,  -1.07022,   6.11354,  0.672653, 0.0195837,  -1.29116,  -1.88799,   1.56784,  0.109339,   1.38196,  -1.47708,
  2.61621, -0.442813,  0.421821,  -2.29765,  0.508448,  -4.35496,  0.672653,   8.07502,  0.282038, -0.420808,  -5.54147,   0.77075,  -3.05202,   3.72445,  -3.22187,
  2.27235,   1.48224,   1.83664,  0.704136,  0.955693,  0.785995, 0.0195837,  0.282038,   5.28125,  -2.15568,  0.895149,  0.498107,   2.73027, -0.909933,  0.401474,
  1.10791, -0.463591,  0.745006,   1.51915,  -1.79853,  -1.06488,  -1.29116, -0.420808,  -2.15568,   6.21088,  0.326835,   1.15671, -0.800233,  0.354842,  0.416901,
-0.637078,   1.19463,   1.05798,   3.82437,  -1.60548,   3.11436,  -1.88799,  -5.54147,  0.895149,  0.326835,   5.92816,  -1.05526,   2.97602,  -3.07508,   2.56724,
 0.328422, -0.623509, -0.867287,  0.499508,  -0.66799,  -1.53233,   1.56784,   0.77075,  0.498107,   1.15671,  -1.05526,   3.65259,  0.725133,  0.440484,  0.606265,
 0.527314,   0.13328,   1.11493,   2.02314, -0.259653,   2.47007,  0.109339,  -3.05202,   2.73027, -0.800233,   2.97602,  0.725133,   3.71439,  -1.84056,   1.86224,
  1.03198, -0.954335,  0.221352,  -1.28832, -0.860349,  -1.65243,   1.38196,   3.72445, -0.909933,  0.354842,  -3.07508,  0.440484,  -1.84056,   3.20605,   -2.1652,
 0.307377, -0.135775, -0.192116,   1.77941,    1.0568,   1.61018,  -1.47708,  -3.22187,  0.401474,  0.416901,   2.56724,  0.606265,   1.86224,   -2.1652,   4.30722;
  viEkfTest.initialiseState(0, state, P);

  // random IMU measurements
  arp::kinematics::ImuMeasurement measurement_0, measurement_1, measurement_2, measurement_3;
  measurement_0.timestampMicroseconds = 0;
  measurement_0.omega_S = Eigen::Vector3d(1,-0.5,0.7);
  measurement_0.acc_S = Eigen::Vector3d(-0.8,-0.6, 0.5);
  measurement_1.timestampMicroseconds = 10000;
  measurement_1.omega_S = measurement_0.omega_S + Eigen::Vector3d(0.1,0.2,0.3)*0.01;
  measurement_1.acc_S = measurement_0.acc_S + Eigen::Vector3d(0.1,0.2,0.3)*0.01;
  measurement_2.timestampMicroseconds = 20000;
  measurement_2.omega_S = measurement_0.omega_S + Eigen::Vector3d(0.3,0.4,0.5)*0.01;
  measurement_2.acc_S = measurement_0.acc_S + Eigen::Vector3d(0.3,0.4,0.5)*0.01;
  measurement_3.timestampMicroseconds = 30000;
  measurement_3.omega_S = measurement_0.omega_S + Eigen::Vector3d(0.5,0.6,0.7)*0.01;
  measurement_3.acc_S = measurement_0.acc_S + Eigen::Vector3d(0.5,0.6,0.7)*0.01;

  // add them
  viEkfTest.addImuMeasurement(measurement_0.timestampMicroseconds,
      measurement_0.omega_S,measurement_0.acc_S);
  viEkfTest.addImuMeasurement(measurement_1.timestampMicroseconds,
      measurement_1.omega_S,measurement_1.acc_S); 
  viEkfTest.addImuMeasurement(measurement_2.timestampMicroseconds,
      measurement_2.omega_S,measurement_2.acc_S);
  viEkfTest.addImuMeasurement(measurement_3.timestampMicroseconds,
      measurement_3.omega_S,measurement_3.acc_S);
 
  // apply the propagation
  viEkfTest.predict(0, 20001);

  // get the result
  Eigen::Matrix<double, 15, 15> newP;
  arp::kinematics::RobotState newState;
  viEkfTest.getState(0,newState,&newP);

  // with own computation:
  arp::kinematics::RobotState newState2;
  newState2.r_W << 0.856562,   1.61761,   2.67016;
  newState2.q_WS.coeffs() << 0.853628, -0.414882, 0.0963025, -0.299862;
  newState2.v_W << -2.25908,  0.7198,    1.3482;
  newState2.b_g << 3.82437,  0.499508,   2.02314;
  newState2.b_a << -1.28832,   1.77941,  0.307377;

  EXPECT_TRUE((newState.r_W-newState2.r_W).norm()<1.0e-4);
  EXPECT_TRUE((newState.q_WS.coeffs()-newState2.q_WS.coeffs()).norm()<1.0e-4);
  EXPECT_TRUE((newState.v_W-newState2.v_W).norm()<1.0e-4);
  EXPECT_TRUE((newState.b_g-newState2.b_g).norm()<1.0e-4);
  EXPECT_TRUE((newState.b_a-newState2.b_a).norm()<1.0e-4);

}

TEST(ViEkfTest, updateState) {

  // construct updateable ekf
  ViEkfTest viEkfTest;

  // arbitrary extrinsics
  arp::kinematics::Transformation T_SC(Eigen::Vector3d(1,2,3),Eigen::Quaterniond(1,2,3,4).normalized());
  viEkfTest.setCameraExtrinsics(T_SC);

  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::NoDistortion>::testObject();
  viEkfTest.setCameraIntrinsics(pinholeCamera);

  // initialise random state
  arp::kinematics::RobotState state;
  state.r_W << 0.902131,   1.60338,   2.64148;
  state.q_WS.coeffs() << 5.2336,  -2.66092,  0.702473,  -2.00367;
  state.q_WS.normalize();
  state.v_W << -2.29765,  0.704136,   1.51915;
  state.b_g << 3.82437,  0.499508,   2.02314;
  state.b_a << -1.28832,   1.77941,  0.307377;
  Eigen::Matrix<double, 15, 15> P;
  P <<   
  6.84348,  0.317284,   1.42921,  0.902131,  -1.09333, -0.254766,   -3.2244,   2.61621,   2.27235,   1.10791, -0.637078,  0.328422,  0.527314,   1.03198,  0.307377,
 0.317284,   3.58218,   1.53043,   1.60338, -0.287659, -0.337079,  -1.24307, -0.442813,   1.48224, -0.463591,   1.19463, -0.623509,   0.13328, -0.954335, -0.135775,
  1.42921,   1.53043,   4.36309,   2.64148, -0.485696,  0.706116,  -1.17031,  0.421821,   1.83664,  0.745006,   1.05798, -0.867287,   1.11493,  0.221352, -0.192116,
 0.902131,   1.60338,   2.64148,    5.2336,  -2.66092,  0.702473,  -2.00367,  -2.29765,  0.704136,   1.51915,   3.82437,  0.499508,   2.02314,  -1.28832,   1.77941,
 -1.09333, -0.287659, -0.485696,  -2.66092,    4.9174,  0.450124, -0.525324,  0.508448,  0.955693,  -1.79853,  -1.60548,  -0.66799, -0.259653, -0.860349,    1.0568,
-0.254766, -0.337079,  0.706116,  0.702473,  0.450124,   4.30821,  -1.07022,  -4.35496,  0.785995,  -1.06488,   3.11436,  -1.53233,   2.47007,  -1.65243,   1.61018,
  -3.2244,  -1.24307,  -1.17031,  -2.00367, -0.525324,  -1.07022,   6.11354,  0.672653, 0.0195837,  -1.29116,  -1.88799,   1.56784,  0.109339,   1.38196,  -1.47708,
  2.61621, -0.442813,  0.421821,  -2.29765,  0.508448,  -4.35496,  0.672653,   8.07502,  0.282038, -0.420808,  -5.54147,   0.77075,  -3.05202,   3.72445,  -3.22187,
  2.27235,   1.48224,   1.83664,  0.704136,  0.955693,  0.785995, 0.0195837,  0.282038,   5.28125,  -2.15568,  0.895149,  0.498107,   2.73027, -0.909933,  0.401474,
  1.10791, -0.463591,  0.745006,   1.51915,  -1.79853,  -1.06488,  -1.29116, -0.420808,  -2.15568,   6.21088,  0.326835,   1.15671, -0.800233,  0.354842,  0.416901,
-0.637078,   1.19463,   1.05798,   3.82437,  -1.60548,   3.11436,  -1.88799,  -5.54147,  0.895149,  0.326835,   5.92816,  -1.05526,   2.97602,  -3.07508,   2.56724,
 0.328422, -0.623509, -0.867287,  0.499508,  -0.66799,  -1.53233,   1.56784,   0.77075,  0.498107,   1.15671,  -1.05526,   3.65259,  0.725133,  0.440484,  0.606265,
 0.527314,   0.13328,   1.11493,   2.02314, -0.259653,   2.47007,  0.109339,  -3.05202,   2.73027, -0.800233,   2.97602,  0.725133,   3.71439,  -1.84056,   1.86224,
  1.03198, -0.954335,  0.221352,  -1.28832, -0.860349,  -1.65243,   1.38196,   3.72445, -0.909933,  0.354842,  -3.07508,  0.440484,  -1.84056,   3.20605,   -2.1652,
 0.307377, -0.135775, -0.192116,   1.77941,    1.0568,   1.61018,  -1.47708,  -3.22187,  0.401474,  0.416901,   2.56724,  0.606265,   1.86224,   -2.1652,   4.30722;
  viEkfTest.initialiseState(0, state, P);

  // create a random visible point in the camera coordinate frame C
  Eigen::Vector3d point_C(1.2, 0.85,  2.14);
  Eigen::Vector4d hp_C;
  hp_C.head<3>() = point_C;
  hp_C[3] = 1.0;

  // project
  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point_C,&imagePoint);

  // now we can set the target
  arp::kinematics::Transformation T_WTj(Eigen::Vector3d(3,2,1), Eigen::Quaterniond(7,8,-9,-4).normalized());
  T_WTj.set((arp::kinematics::Transformation(state.r_W,state.q_WS)*T_SC*hp_C).head<3>(),T_WTj.q());
  viEkfTest.setTarget(0, T_WTj, 0.0); // target size 0 so all four corners are the same, i.e. point_W
  
  // apply the update
  Eigen::Matrix<double,2,4> imagePointMeasurements;
  imagePointMeasurements.col(2) = imagePoint + Eigen::Vector2d(0.1,-0.1);
  viEkfTest.update(0, imagePointMeasurements, 0, 2);

  // get the result
  Eigen::Matrix<double, 15, 15> newP;
  arp::kinematics::RobotState newState;
  viEkfTest.getState(0, newState, &newP);

  // with own computation:
  arp::kinematics::RobotState newState2;
  newState2.r_W <<  0.902192, 1.60335,  2.64146;
  newState2.q_WS.coeffs() << 0.83826, -0.426184,  0.112569, -0.320961;
  newState2.v_W << -2.29762,  0.704153,  1.51912;
  newState2.b_g <<  3.82441,  0.499508,  2.02315;
  newState2.b_a << -1.28833,  1.77946,  0.307319;

  EXPECT_TRUE((newState.r_W-newState2.r_W).norm()<2.0e-5);
  EXPECT_TRUE((newState.q_WS.coeffs()-newState2.q_WS.coeffs()).norm()<2.0e-5);
  EXPECT_TRUE((newState.v_W-newState2.v_W).norm()<2.0e-5);
  EXPECT_TRUE((newState.b_g-newState2.b_g).norm()<2.0e-5);
  EXPECT_TRUE((newState.b_a-newState2.b_a).norm()<2.0e-5);
}

} // namespace

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


