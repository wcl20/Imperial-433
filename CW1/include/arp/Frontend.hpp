/*
 * Frontend.hpp
 *
 *  Created on: 8 Dec 2016
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <AprilTags/TagDetector.h>
#include <AprilTags/TagDetection.h>
#include <AprilTags/Tag36h11.h>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/cameras/NoDistortion.hpp>
#include <arp/kinematics/Transformation.hpp>

namespace arp {

///\brief This class processes an image and returns the detected marker poses.
class Frontend
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Define the tag (id) we are supposed to see along with their metric sizes.
  bool setTarget(unsigned int id, double targetSizeMeters);

  /// \brief Sets the underlying camera parameters (RadialTangentialDistortion)
  void setCameraParameters(int imageWidth, int imageHeight, double focalLengthU,
                           double focalLengthV, double imageCenterU,
                           double imageCenterV, double k1, double k2, double p1,
                           double p2);

  /// \brief Destructor
  ~Frontend();

  /// \brief The undistorted camera model used for the estimator.
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> undistortedCameraModel() const;

  /// \brief A simple struct containing all the necessary information about a
  ///        tag detection.
  struct Detection {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    kinematics::Transformation T_CT; ///< The pose of the camera relative to the tag.
    Eigen::Matrix<double,2,4> points; ///< The tag corner points in pixels [u1...u4; v1...v4].
    int id; ///< The ID of the detected tag.
    // void draw(cv::Mat& image) const;
    AprilTags::TagDetection tagdetector;
  };
  typedef std::vector<Detection, Eigen::aligned_allocator<Detection>> DetectionVec;

  /// \brief Detect the tags in an image and obtain measurements that can be
  ///        fed to an estimator.
  int detect(const cv::Mat& image, DetectionVec & detections);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>* camera_ = nullptr;

 protected:
  AprilTags::TagDetector tagDetector_ = AprilTags::TagDetector(
      AprilTags::tagCodes36h11);
  std::map<int,double> idToSize_;
};

}  // namespace arp

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_FRONTEND_HPP_ */
