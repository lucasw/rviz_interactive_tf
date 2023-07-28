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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
  server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_tf"));
  pose_.orientation.w = 1.0;

  // TODO(lucasw) need way to get parameters out- tf echo would work
  float scale_ = 1.0;
  ros::param::get("~scale", scale_);
  ros::param::get("~parent_frame", parent_frame_);
  ros::param::get("~frame", frame_);

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

  // initial pose setting
  double x = 0.0;
  ros::param::get("~x", x);
  double y = 0.0;
  ros::param::get("~y", y);
  double z = 0.0;
  ros::param::get("~z", z);

  pose_.position.x = x;
  pose_.position.y = y;
  pose_.position.z = z;

  double roll = 0.0;
  ros::param::get("~roll", roll);
  double pitch = 0.0;
  ros::param::get("~pitch", pitch);
  double yaw = 0.0;
  ros::param::get("~yaw", yaw);

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  pose_.orientation = tf2::toMsg(quat);

  server_->setPose(int_marker_.name, pose_, int_marker_.header);
  server_->applyChanges();
}

InteractiveTf::~InteractiveTf()
{
  server_.reset();
}

void InteractiveTf::updateTf(int, const ros::TimerEvent& event)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose_.position.x, pose_.position.y, pose_.position.z));
  transform.setRotation(tf::Quaternion(pose_.orientation.x,
      pose_.orientation.y,
      pose_.orientation.z,
      pose_.orientation.w));
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
      parent_frame_, frame_));
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
