// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
#include "arp/kinematics/Transformation.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

// Test the projection and unprojection
TEST(PinholeCamera, projectBackProject)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();

  // project
  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point_C,&imagePoint);

  // backProject
  Eigen::Vector3d ray_C;
  pinholeCamera.backProject(imagePoint,&ray_C);

  // now they should align:
  EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);
}

// TODO: write more tests here...

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

