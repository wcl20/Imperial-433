#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/Frontend.hpp>
// #include <arp/InteractiveMarkerServer.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/VisualInertialTracker.hpp>

// #define _USE_MATH_DEFINES
// #include <math.h>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    visualInertialTracker_->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;

    // Get Angular velocity measurement
    Eigen::Vector3d omega_S;
    float omega_x = msg->angular_velocity.x;
    float omega_y = msg->angular_velocity.y;
    float omega_z = msg->angular_velocity.z;
    omega_S << omega_x, omega_y, omega_z;

    // Get linear acceleration measurement
    Eigen::Vector3d acc_S;
    float acc_x = msg->linear_acceleration.x;
    float acc_y = msg->linear_acceleration.y;
    float acc_z = msg->linear_acceleration.z;
    acc_S << acc_x, acc_y, acc_z;

    visualInertialTracker_->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
  }

  void setVisualInertialTracker(arp::VisualInertialTracker* visualInertialTracker)
  {
    visualInertialTracker_ = visualInertialTracker;
  }

 private:
  arp::VisualInertialTracker* visualInertialTracker_;
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  bool flyChallenge = false;
  bool isReturn = false;
  bool landed = false;

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // set up autopilot
  arp::Autopilot autopilot(nh);
  // arp::InteractiveMarkerServer markerServer(autopilot);

  // Camera parameters
  double fv, fu, cv, cu, k1, k2, p1, p2;
  nh.getParam("arp_node/fu", fu);
  nh.getParam("arp_node/fv", fv);
  nh.getParam("arp_node/cu", cu);
  nh.getParam("arp_node/cv", cv);
  nh.getParam("arp_node/k1", k1);
  nh.getParam("arp_node/k2", k2);
  nh.getParam("arp_node/p1", p1);
  nh.getParam("arp_node/p2", p2);

  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);

  // set up target. Change from identity to vertical, if needed!
  Eigen::Matrix4d T_WT;
  // T_WT.setIdentity();
  T_WT << 1, 0, 0, 0,
          0, 0, -1, 0,
          0, 1, 0, 0,
          0, 0, 0, 1;

  // set up Frontend
  arp::Frontend frontend;
  frontend.setCameraParameters(640, 360, fu, fv, cu, cv, k1, k2, p1, p2);

  // set up EKF
  arp::ViEkf estimator;
  // estimator.setTarget(0, arp::kinematics::Transformation(T_WT), 0.1675);
  Eigen::Matrix4d T_SC_mat;
  T_SC_mat << -0.00195087, -0.03257782, 0.99946730, 0.17409445,
              -0.99962338, -0.02729525, -0.00284087, 0.02255834,
              0.02737326, -0.99909642, -0.03251230, 0.00174723,
              0.00000000, 0.00000000, 0.00000000, 1.00000000;
  arp::kinematics::Transformation T_SC(T_SC_mat);
  estimator.setCameraExtrinsics(T_SC);
  estimator.setCameraIntrinsics(frontend.undistortedCameraModel());

  // set up visual-inertial tracking
  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(estimator);
  subscriber.setVisualInertialTracker(&visualInertialTracker);

  // set up visualisation
  visualInertialTracker.setVisualisationCallback(std::bind(
  &arp::StatePublisher::publish, &pubState, std::placeholders::_1,
  std::placeholders::_2));
  // set up controller
  visualInertialTracker.setControllerCallback(std::bind(
  &arp::Autopilot::controllerCallback, &autopilot,
  std::placeholders::_1, std::placeholders::_2));

  // init frontend and estimator targets
  Eigen::Matrix3d C_WT;
  C_WT << 0.000000000000000, 0.000000000000000, 1.000000000000000,
  1.000000000000000, 0.000000000000000, 0.000000000000000,
  0.000000000000000, 1.000000000000000, 0.000000000000000;
  Eigen::Quaterniond q_WT(C_WT);
  typedef arp::kinematics::Transformation Transformation;
  Transformation T_WT0(Eigen::Vector3d(0, 2.220, 1.656), q_WT);
  frontend.setTarget(0, 0.1725);
  estimator.setTarget(0, T_WT0, 0.1725);
  Transformation T_WT1(Eigen::Vector3d(0, 2.997, 1.656), q_WT);
  frontend.setTarget(1, 0.1725);
  estimator.setTarget(1, T_WT1, 0.1725);
  Transformation T_WT2(Eigen::Vector3d(0, 3.224, 1.656), q_WT);
  frontend.setTarget(2, 0.1725);
  estimator.setTarget(2, T_WT2, 0.1725);
  Transformation T_WT3(Eigen::Vector3d(0, 4.000, 1.657), q_WT);
  frontend.setTarget(3, 0.1725);
  estimator.setTarget(3, T_WT3, 0.1725);
  Transformation T_WT4(Eigen::Vector3d(0, 4.227, 1.661), q_WT);
  frontend.setTarget(4, 0.1725);
  estimator.setTarget(4, T_WT4, 0.1725);
  Transformation T_WT5(Eigen::Vector3d(0, 5.000, 1.654), q_WT);
  frontend.setTarget(5, 0.1725);
  estimator.setTarget(5, T_WT5, 0.1725);
  Transformation T_WT6(Eigen::Vector3d(0, 5.660, 1.657), q_WT);
  frontend.setTarget(6, 0.1725);
  estimator.setTarget(6, T_WT6, 0.1725);
  Transformation T_WT7(Eigen::Vector3d(0, 6.420, 1.657), q_WT);
  frontend.setTarget(7, 0.1725);
  estimator.setTarget(7, T_WT7, 0.1725);
  Transformation T_WT8(Eigen::Vector3d(0, 7.195, 1.648), q_WT);
  frontend.setTarget(8, 0.1610);
  estimator.setTarget(8, T_WT8, 0.1610);
  Transformation T_WT10(Eigen::Vector3d(0, 2.221, 0.955), q_WT);
  frontend.setTarget(10, 0.1610);
  estimator.setTarget(10, T_WT10, 0.1610);
  Transformation T_WT11(Eigen::Vector3d(0, 2.997, 0.957), q_WT);
  frontend.setTarget(11, 0.1725);
  estimator.setTarget(11, T_WT11, 0.1725);
  Transformation T_WT12(Eigen::Vector3d(0, 3.224, 0.956), q_WT);
  frontend.setTarget(12, 0.1725);
  estimator.setTarget(12, T_WT12, 0.1725);
  Transformation T_WT13(Eigen::Vector3d(0, 3.999, 0.955), q_WT);
  frontend.setTarget(13, 0.1610);
  estimator.setTarget(13, T_WT13, 0.1610);
  Transformation T_WT14(Eigen::Vector3d(0, 4.226, 0.956), q_WT);
  frontend.setTarget(14, 0.1610);
  estimator.setTarget(14, T_WT14, 0.1610);
  Transformation T_WT15(Eigen::Vector3d(0, 4.998, 0.950), q_WT);
  frontend.setTarget(15, 0.1725);
  estimator.setTarget(15, T_WT15, 0.1725);
  Transformation T_WT16(Eigen::Vector3d(0, 5.660, 0.955), q_WT);
  frontend.setTarget(16, 0.1725);
  estimator.setTarget(16, T_WT16, 0.1725);
  Transformation T_WT17(Eigen::Vector3d(0, 6.415, 0.950), q_WT);
  frontend.setTarget(17, 0.1610);
  estimator.setTarget(17, T_WT17, 0.1610);
  Transformation T_WT18(Eigen::Vector3d(0, 7.189, 0.942), q_WT);
  frontend.setTarget(18, 0.1725);
  estimator.setTarget(18, T_WT18, 0.1725);

  // set up waypoints_
  std::deque<arp::Autopilot::Waypoint> waypoints_AB;
  double posTolerance = 0.2;
  arp::Autopilot::Waypoint start = {2.8, 2.2, 1.0, 3.14159265, 0.1};
  waypoints_AB.push_back(start);
  waypoints_AB.push_back({2.3, 2.2, 1.0, 3.14159265, posTolerance});
  waypoints_AB.push_back({1.8, 2.2, 1.0, 3.14159265, posTolerance});
  //waypoints_AB.push_back({1.8, 2.7, 1.0, 3.14159265, posTolerance});
  waypoints_AB.push_back({1.8, 3.2, 1.0, 3.14159265, posTolerance});
  //waypoints_AB.push_back({1.8, 3.7, 1.0, 3.14159265, posTolerance});
  waypoints_AB.push_back({1.8, 4.2, 1.0, 3.14159265, posTolerance});
  //waypoints_AB.push_back({1.8, 4.7, 1.0, 3.14159265, posTolerance});
  waypoints_AB.push_back({1.8, 5.2, 1.0, 3.14159265, posTolerance});
  //waypoints_AB.push_back({1.8, 5.7, 1.0, 3.14159265, posTolerance});
  waypoints_AB.push_back({1.8, 6.2, 1.0, 3.14159265, posTolerance});
  waypoints_AB.push_back({1.8, 6.5, 1.0, 3.14159265, 0.1});
  waypoints_AB.push_back({2.3, 6.5, 1.0, 3.14159265, 0.1});
  arp::Autopilot::Waypoint end = {2.8, 6.5, 1.0, 3.14159265, 0.1};
  waypoints_AB.push_back(end);

  std::deque<arp::Autopilot::Waypoint> waypoints_BA;
  waypoints_BA.push_back(end);
  waypoints_BA.push_back({2.3, 6.5, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({1.8, 6.5, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({1.8, 6.2, 1.0, 3.14159265, posTolerance});
  //waypoints_BA.push_back({1.8, 5.7, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({1.8, 5.2, 1.0, 3.14159265, posTolerance});
  //waypoints_BA.push_back({1.8, 4.7, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({1.8, 4.2, 1.0, 3.14159265, posTolerance});
  //waypoints_BA.push_back({1.8, 3.7, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({1.8, 3.2, 1.0, 3.14159265, posTolerance});
  //waypoints_BA.push_back({1.8, 2.7, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({1.8, 2.2, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back({2.3, 2.2, 1.0, 3.14159265, posTolerance});
  waypoints_BA.push_back(start);
  // std::copy(waypoints_AB.begin(), waypoints_AB.end(), waypoints_BA.begin());
  // std::reverse(waypoints_BA.begin(), waypoints_BA.end());
  // cout << waypoints_BA << endl;


  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    std::vector<std::string> statusList = {"TakingOff", "Inited", "Landed", "Flying", "Hovering", "Test", "TakingOff", "Flying2", "Landing", "Looping"};
    cv::Mat image;
    if (subscriber.getLastImage(image)) {
      int width = image.size().width;
      int height = image.size().height;

      // Added overlays of battery info, status and instruction to image
      std::string battery = std::to_string((int)autopilot.droneBattery()) + "%";
      std::string status = statusList[autopilot.droneStatus()];
      std::string instruction = "^:forward v:backward <:left >:right W:up S:down A:yaw left D:yaw right";

      // Bounding box for display
      cv::rectangle(image, cv::Point(5, 5), cv::Point(90, 25), cv::Scalar(0, 0, 0, 255), 2);
      cv::rectangle(image, cv::Point(5, height-25), cv::Point(width- 5, height-5), cv::Scalar(0, 0, 0, 255), 2);
      cv::rectangle(image, cv::Point(width - 40, 5), cv::Point(width - 5, 25), cv::Scalar(0, 0, 0, 255), 2);

      // Display background
      cv::rectangle(image, cv::Point(5, 5), cv::Point(90, 25), cv::Scalar(255, 255, 255, 255), -1);
      cv::rectangle(image, cv::Point(5, height-25), cv::Point(width- 5, height-5), cv::Scalar(255, 255, 255, 255), -1);
      cv::rectangle(image, cv::Point(width - 40, 5), cv::Point(width - 5, 25), cv::Scalar(255, 255, 255, 255), -1);

      // Display text
      cv::putText(image, status, cv::Point(5, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 255));
      cv::putText(image, instruction, cv::Point(5, height-10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 255));
      cv::putText(image, battery, cv::Point(width-40, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 255));

      // http://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      //Convert to SDL_Surface
      IplImage opencvimg2 = (IplImage) image;
      IplImage* opencvimg = &opencvimg2;

      auto frameSurface = SDL_CreateRGBSurfaceFrom(
          (void*) opencvimg->imageData, opencvimg->width, opencvimg->height,
          opencvimg->depth * opencvimg->nChannels, opencvimg->widthStep,
          0xff0000, 0x00ff00, 0x0000ff, 0);
      if (frameSurface == NULL) {
        std::cout << "Couldn't convert Mat to Surface." << std::endl;
      } else {
        texture = SDL_CreateTextureFromSurface(renderer, frameSurface);
        SDL_FreeSurface(frameSurface);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
      }
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // Initialise move parameters
    double forward = 0.0, left = 0.0, up = 0.0, rotateLeft = 0.0;

    // check states!
    auto droneStatus = autopilot.droneStatus();
    // command
    if (state[SDL_SCANCODE_RCTRL]) {
      double x, y, z, yaw;
      autopilot.getPoseReference(x, y, z, yaw);
      // markerServer.activate(x, y, z, yaw);

      autopilot.setAutomatic();
    }
    if (state[(SDL_SCANCODE_SPACE)]) {
      flyChallenge = false;
      autopilot.setManual();
    }
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_UP]) {
      // Move the drone forward
      forward += 1.0;
    }
    if (state[SDL_SCANCODE_DOWN]) {
      // Move the drone backward
      forward -= 1.0;
    }
    if (state[SDL_SCANCODE_LEFT]) {
      // Move the drone left
      left += 1.0;
    }
    if (state[SDL_SCANCODE_RIGHT]) {
      // Move the drone right
      left -= 1.0;
    }
    if (state[SDL_SCANCODE_W]) {
      // Move the drone up
      up += 1.0;
    }
    if (state[SDL_SCANCODE_S]) {
      // Move the drone down
      up -= 1.0;
    }
    if (state[SDL_SCANCODE_A]) {
      // Yaw the drone left
      rotateLeft += 1.0;
    }
    if (state[SDL_SCANCODE_D]) {
      // Yaw the drone right
      rotateLeft -= 1.0;
    }
    if (state[SDL_SCANCODE_P]) {
      // Start challenge
      flyChallenge = true;
      autopilot.setAutomatic();
      // Set waypoints from A to B
      // std::cout << "Adding waypoints (A->B)...           status=" << droneStatus << std::endl;
      autopilot.flyPath(waypoints_AB);
    }

    // Check progress of flying challenge
    if (flyChallenge) {
      // Get number of waypoints left
      int waypointsLeft = autopilot.waypointsLeft();
      // std::cout << "Waypoints left=" << waypointsLeft << std::endl;
      // If drone if flying. Start and Take off from B
      if (droneStatus == arp::Autopilot::Flying ||
        droneStatus == arp::Autopilot::Hovering ||
        droneStatus == arp::Autopilot::Flying2 ) {
          // Drone has no more way points when reached A or B
        if (!waypointsLeft) {
          // Land drone
          autopilot.setManual();
          autopilot.manualMove(0, 0, 0, 0);
          // std::cout << "Landing...           status=" << droneStatus << std::endl;
          landed = autopilot.land();
          ros::Duration(2).sleep();
          // Returning from B, set waypoints for returning to A
          // std::cout << "Adding waypoints (B->A)...           status=" << droneStatus << std::endl;
          autopilot.flyPath(waypoints_BA);
          if (landed)
          {
            // Drone land at B
            if (!isReturn) {
              isReturn = true;
            } else {
              // Drone land at A
              flyChallenge = false;
              isReturn = false;
              autopilot.setManual();
            }
          }
        }
      }
      // If drone is landed. At B and End.
      if (droneStatus == arp::Autopilot::Landed || landed) {
        // Drone is landed at B
        if (isReturn) {
          // std::cout << "Taking off from B...           status=" << droneStatus << std::endl;
          bool success = autopilot.takeoff();
          landed = !success;
          ros::Duration(4).sleep();
          // std::cout << "Setting automatic ..." << std::endl;
          autopilot.setAutomatic();
          ros::Duration(2).sleep();

        }
      }
    }

    // Assembles several keys pressed into one velocity
    // If no keys are pressed, send all zeros to stop the drone from moving
    bool success = autopilot.manualMove(forward, left, up, rotateLeft);
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
