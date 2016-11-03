/*
 * Copyright (c) 2016, Lucas Walter
 *
 * Create a tf in rviz with a specified parent and name,
 * drag it around and rotate it with interactive marker controls.
 */

#include <boost/bind.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>

void testFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{}
class InteractiveTf
{
  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void processFeedback(unsigned ind, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  visualization_msgs::InteractiveMarker int_marker_;

  void updateTf(int, const ros::TimerEvent& event);
  ros::Timer tf_timer_;
public:
  InteractiveTf();
  ~InteractiveTf();
};

InteractiveTf::InteractiveTf()
{
  server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_tf"));

  {
  visualization_msgs::InteractiveMarkerControl control;

  int_marker_.header.frame_id = "map";
  int_marker_.header.stamp = ros::Time::now();
	int_marker_.name = "interactive_tf";
	int_marker_.description = "control a tf with 6dof";

  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  control.always_visible = true;
  control.markers.push_back(box_marker);
  int_marker_.controls.push_back(control);
  }

  {
  visualization_msgs::InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);
  }

	server_->insert(int_marker_);
  // Can't seem to get rid of the 0, _1 parameter
	server_->setCallback(int_marker_.name,
      boost::bind(&InteractiveTf::processFeedback, this, 0, _1));
	// server_->setCallback(int_marker_.name, testFeedback);

  server_->applyChanges();

  tf_timer_ = nh_.createTimer(ros::Duration(0.1),
      boost::bind(&InteractiveTf::updateTf, this, 0, _1));
}

InteractiveTf::~InteractiveTf()
{
  server_.reset();
}

void InteractiveTf::updateTf(int, const ros::TimerEvent& event)
{

}

void InteractiveTf::processFeedback(
    unsigned ind,
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM(feedback->control_name);
  ROS_INFO_STREAM(feedback->event_type);
  ROS_INFO_STREAM(feedback->mouse_point);
	// TODO(lucasw) all the pose changes get handled by the server elsewhere?
	server_->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_tf");
  InteractiveTf interactive_tf;
  ros::spin();
  return 0;
}
