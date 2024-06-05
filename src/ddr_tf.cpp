/*
 * Copyright (c) 2016, Lucas Walter
 *
 * Create a tf in rviz with a specified parent and name,
 * drag it around and rotate it with interactive marker controls.
 */

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <geometry_msgs/Pose.h>
#include <memory>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/InteractiveMarker.h>

class DdrTf
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  tf::TransformBroadcaster br_;

  void updateTf(int, const ros::TimerEvent& event);
  ros::Timer tf_timer_;

  std::string frame_id_ = "odom";
  std::string child_frame_id_ = "ddr_tf";

  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;

  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;

public:
  DdrTf();
  ~DdrTf();
};

DdrTf::DdrTf() :
  private_nh_("~")
{
  ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(private_nh_);


  double scale = 10.0;
  ros::param::get("~scale", scale);
  double sc = 100.0;

  const std::string frame_id_desc = "frame id";
  ddr_->registerVariable<std::string>("frame_id", &frame_id_, frame_id_desc);
  const std::string child_frame_id_desc = "child_frame id";
  ddr_->registerVariable<std::string>("child_frame_id", &child_frame_id_, child_frame_id_desc);

  const std::string x_desc = "x position";
  ddr_->registerVariable<double>("x", &x_, x_desc, -sc * scale, sc * scale);
  const std::string y_desc = "y position";
  ddr_->registerVariable<double>("y", &y_, y_desc, -sc * scale, sc * scale);
  const std::string z_desc = "z position";
  ddr_->registerVariable<double>("z", &z_, z_desc, -sc * scale, sc * scale);

  const std::string roll_desc = "roll angle rdians";
  ddr_->registerVariable<double>("roll", &roll_, roll_desc, -sc * scale, sc * scale);
  const std::string pitch_desc = "pitch angle rdians";
  ddr_->registerVariable<double>("pitch", &pitch_, pitch_desc, -sc * scale, sc * scale);
  const std::string yaw_desc = "yaw angle rdians";
  ddr_->registerVariable<double>("yaw", &yaw_, yaw_desc, -sc * scale, sc * scale);

  ddr_->publishServicesTopics();

  tf_timer_ = nh_.createTimer(ros::Duration(0.05),
      boost::bind(&DdrTf::updateTf, this, 0, boost::placeholders::_1));
}

DdrTf::~DdrTf()
{
}

void DdrTf::updateTf(int, const ros::TimerEvent& event)
{
  tf::Quaternion quat;
  quat.setRPY(roll_, pitch_, yaw_);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x_, y_, z_));
  transform.setRotation(quat);

  br_.sendTransform(tf::StampedTransform(transform, event.current_expected, frame_id_, child_frame_id_));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_tf");
  DdrTf interactive_tf;
  ros::spin();
  return 0;
}
