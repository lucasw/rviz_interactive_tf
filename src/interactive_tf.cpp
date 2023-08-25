/*
 * Copyright (c) 2016, Lucas Walter
 *
 * Create a tf in rviz with a specified parent and name,
 * drag it around and rotate it with interactive marker controls.
 */

#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
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

  geometry_msgs::Pose pose_;
  tf::TransformBroadcaster br_;
  tf2_ros::StaticTransformBroadcaster static_br_;
  std::string parent_frame_;
  std::string frame_;
  void updateTf(int, const ros::TimerEvent& event);
  ros::Timer tf_timer_;

public:
  InteractiveTf();
  ~InteractiveTf();
};

InteractiveTf::InteractiveTf() :
  parent_frame_("map"),
  frame_("interactive_tf")
{
  server_.reset(new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));

  // TODO(lucasw) need way to get parameters out- tf echo would work
  float scale_ = 1.0;
  ros::param::param<float>("~scale", scale_, 1.0);
  ros::param::param<std::string>("~parent_frame", parent_frame_, "world");
  ros::param::param<std::string>("~frame", frame_, "interactive_tf");
  ros::param::param<double>("~initial_pos_x", pose_.position.x, 0.0);
  ros::param::param<double>("~initial_pos_y", pose_.position.y, 0.0);
  ros::param::param<double>("~initial_pos_z", pose_.position.z, 0.0);
  ros::param::param<double>("~initial_rot_x", pose_.orientation.x, 0.0);
  ros::param::param<double>("~initial_rot_y", pose_.orientation.y, 0.0);
  ros::param::param<double>("~initial_rot_z", pose_.orientation.z, 0.0);
  ros::param::param<double>("~initial_rot_w", pose_.orientation.w, 1.0);

  int_marker_.header.frame_id = parent_frame_;
  // http://answers.ros.org/question/262866/interactive-marker-attached-to-a-moving-frame/
  // putting a timestamp on the marker makes it not appear
  // int_marker_.header.stamp = ros::Time::now();
  int_marker_.name = "interactive_tf";
  int_marker_.description = "control a tf with 6dof";
  int_marker_.pose = pose_;
  int_marker_.scale = scale_;

  {
  visualization_msgs::InteractiveMarkerControl control;

  // TODO(lucasw) get roll pitch yaw and set as defaults
  
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_x";
  // TODO(lucasw) how to set initial values?
  // double x = 0.0;
  // ros::param::get("~x", x);
  // control.pose.position.x = x;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);
  // control.pose.position.x = 0.0;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_z";
  // double z = 0.0;
  // control.pose.position.z = ros::param::get("~z", z);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);
  // control.pose.position.z = 0.0;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_y";
  // double y = 0.0;
  // control.pose.position.z = ros::param::get("~y", y);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);
  // control.pose.position.y = 0.0;
  }

  server_->insert(int_marker_);
  server_->setCallback(int_marker_.name,
      boost::bind(&InteractiveTf::processFeedback, this, 0, boost::placeholders::_1));
  // server_->setCallback(int_marker_.name, testFeedback);

  server_->applyChanges();

  tf_timer_ = nh_.createTimer(ros::Duration(0.05),
      boost::bind(&InteractiveTf::updateTf, this, 0, boost::placeholders::_1));
}

InteractiveTf::~InteractiveTf()
{
  server_.reset();
}

void InteractiveTf::updateTf(int, const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.child_frame_id = frame_;
  transform_msg.header.frame_id = parent_frame_;
  transform_msg.header.stamp = ros::Time::now();
  transform_msg.transform.translation.x = pose_.position.x;
  transform_msg.transform.translation.y = pose_.position.y;
  transform_msg.transform.translation.z = pose_.position.z;
  transform_msg.transform.rotation.x = pose_.orientation.x;
  transform_msg.transform.rotation.y = pose_.orientation.y;
  transform_msg.transform.rotation.z = pose_.orientation.z;
  transform_msg.transform.rotation.w = pose_.orientation.w;
  static_br_.sendTransform(transform_msg);
}

void InteractiveTf::processFeedback(
    unsigned ind,
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM(feedback->header.frame_id);
  pose_ = feedback->pose;
  ROS_DEBUG_STREAM(feedback->control_name);
  ROS_DEBUG_STREAM(feedback->event_type);
  ROS_DEBUG_STREAM(feedback->mouse_point);
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
