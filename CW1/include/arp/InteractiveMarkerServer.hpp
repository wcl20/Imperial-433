/*
 * InteractiveMarkerServer.hpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_INTERACTIVEMARKERSERVER_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_INTERACTIVEMARKERSERVER_HPP_

#include <std_srvs/Empty.h>
#include <interactive_markers/interactive_marker_server.h>
#include <arp/Autopilot.hpp>

namespace arp {

class InteractiveMarkerServer
{
public:
  InteractiveMarkerServer(arp::Autopilot & autopilot) : autopilot_(&autopilot)
  {
    // create an interactive marker server on the topic namespace simple_marker
    server_.reset(
      new interactive_markers::InteractiveMarkerServer("reference"));
  }

  void activate(double x, double y, double z, double yaw) {
    deactivate(); // make sure not called more than once...
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "my_marker";
    int_marker.description = "Drone reference pose";
    int_marker.scale = 0.5;

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // set the pose to the requested one
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = sin(yaw / 2.0);
    int_marker.pose.orientation.w = cos(yaw / 2.0);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // set up quadrotor style controls
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_->insert(
      int_marker,
      std::bind(&InteractiveMarkerServer::processFeedback, this,
                std::placeholders::_1));

    // 'commit' changes and send to all clients
    server_->applyChanges();
  }
  void deactivate() {
    // get rid of markers
    server_->clear();

    // 'commit' changes and send to all clients
    server_->applyChanges();
  }
  void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    // compute yaw
    const double angle = 2.0 * acos(feedback->pose.orientation.w);
    double yaw = angle;
    if (feedback->pose.orientation.z < 0.0) {
      yaw = -angle;
    }
    autopilot_->setPoseReference(feedback->pose.position.x, feedback->pose.position.y,
                                 feedback->pose.position.z, yaw);
  }
protected:
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  arp::Autopilot* autopilot_;
};

} // namespace arp

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_INTERACTIVEMARKERSERVER_HPP_ */
