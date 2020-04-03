/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2016
 *      Author: sleutene
 */

#include <arp/Frontend.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace arp {

Frontend::~Frontend() {
  if(camera_ != nullptr) {
    delete camera_;
  }
}

void Frontend::setCameraParameters(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2)
{
  if(camera_ != nullptr) {
    delete camera_;
  }
  camera_ = new arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV, arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2));
  camera_->initialiseUndistortMaps();
}

// the undistorted camera model used for the estimator (later)
arp::cameras::PinholeCamera<arp::cameras::NoDistortion>
    Frontend::undistortedCameraModel() const {
  assert(camera_);
  return camera_->undistortedPinholeCamera();
}

int Frontend::detect(const cv::Mat& image, DetectionVec & detections)
{
  // Convert to greyscale
  cv::Mat greyscaleImage;
  cv::cvtColor(image, greyscaleImage, CV_BGR2GRAY);
  // Undistort input image
  cv::Mat undistortImage;
  bool success = camera_->undistortImage(greyscaleImage, undistortImage);
  // Extract AprilTags
  std::vector<AprilTags::TagDetection> aprilTags = tagDetector_.extractTags(undistortImage);
  // Iterate detections
  for (std::vector<AprilTags::TagDetection>::iterator it = aprilTags.begin(); it != aprilTags.end(); ++it) {
    // Check id is registered
    int id = it->id;
    std::map<int, double>::iterator mapit = idToSize_.find(id);
    if (mapit != idToSize_.end()) {
      // Create new detection struct
      struct Detection detection;
      // Fill detection id
      detection.id = id;
      // Fill detection transformation
      double tagSize = mapit->second;
      double fu = undistortedCameraModel().focalLengthU();
      double fv = undistortedCameraModel().focalLengthV();
      double cu = undistortedCameraModel().imageCenterU();
      double cv = undistortedCameraModel().imageCenterV();
      Eigen::Matrix4d transformMatrix = it->getRelativeTransform(tagSize, fu, fv, cu, cv);
      kinematics::Transformation transformation(transformMatrix);
      detection.T_CT = transformation;
      // Fill detection corners
      Eigen::Matrix<double,2,4> points;
      std::pair<float,float> p[4] = it->p;
      for (unsigned int i = 0; i < 4; i++) {
        points(0,i) = p[i].first;
        points(1,i) = p[i].second;
      }
      detection.points = points;
      detection.tagdetector = *it;
      // Append detection to list of detections
      detections.push_back(detection);
    }
  }
  // Return number of detections
  return detections.size();
}

bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp
