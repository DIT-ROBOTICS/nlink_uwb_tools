/**
 *
 * @file nlink_tagframe_converter.cpp
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2022-03-23
 *
 */

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <nlink_parser/LinktrackTagframe0.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher pub_imu;
ros::Publisher pub_pose;
ros::Publisher pub_gnss;
nlink_parser::LinktrackTagframe0 input_tag;
sensor_msgs::Imu output_imu;
geometry_msgs::PoseWithCovarianceStamped output_pose;
// sensor_msgs::NavSatFix output_gnss;

bool p_active;
std::string p_map_frame_id;
std::string p_frame_id;
double p_cutoff_freq;
std::vector<double> p_linear_accel_cov;
std::vector<double> p_angular_vel_cov;
std::vector<double> p_pose_cov;

double lpf_gain;
std::vector<double> lpf_pose(3);

void tagCallback(const nlink_parser::LinktrackTagframe0::ConstPtr& ptr)
{
  ros::Time now = ros::Time::now();
  input_tag = *ptr;

  lpf_pose.at(0) += lpf_gain * (input_tag.pos_3d[0] - lpf_pose.at(0));
  lpf_pose.at(1) += lpf_gain * (input_tag.pos_3d[1] - lpf_pose.at(1));
  lpf_pose.at(2) += lpf_gain * (input_tag.pos_3d[2] - lpf_pose.at(2));

  output_imu.header.stamp = now;
  output_imu.header.frame_id = p_frame_id;
  output_imu.linear_acceleration.x = input_tag.imu_acc_3d[0];
  output_imu.linear_acceleration.y = input_tag.imu_acc_3d[1];
  output_imu.linear_acceleration.z = input_tag.imu_acc_3d[2];
  for (int i = 0; i < p_linear_accel_cov.size(); i++)
  {
    output_imu.linear_acceleration_covariance[i] = p_linear_accel_cov[i];
  }

  output_imu.angular_velocity.x = input_tag.imu_gyro_3d[0];
  output_imu.angular_velocity.y = input_tag.imu_gyro_3d[1];
  output_imu.angular_velocity.z = input_tag.imu_gyro_3d[2];
  for (int i = 0; i < p_angular_vel_cov.size(); i++)
  {
    output_imu.angular_velocity_covariance[i] = p_angular_vel_cov[i];
  }
  pub_imu.publish(output_imu);

  output_imu.orientation.w = input_tag.quaternion[0];
  output_imu.orientation.x = input_tag.quaternion[1];
  output_imu.orientation.y = input_tag.quaternion[2];
  output_imu.orientation.z = input_tag.quaternion[3];
  output_imu.orientation_covariance[0] = -1.0;

  output_pose.header.frame_id = p_map_frame_id;
  output_pose.header.stamp = now;

  

  output_pose.pose.pose.position.x = lpf_pose.at(0);
  output_pose.pose.pose.position.y = lpf_pose.at(1);
  output_pose.pose.pose.position.z = lpf_pose.at(2);

  output_pose.pose.pose.orientation = output_imu.orientation;

  for (int i = 0; i < p_pose_cov.size(); i++)
  {
    output_pose.pose.covariance[i] = p_pose_cov[i];
  }

  pub_pose.publish(output_pose);

  // output_gnss.header.frame_id = p_map_frame_id;
  // output_gnss.header.stamp = now;

  // output_gnss.status.status = 2;
  // output_gnss.status.service = 1;

  // output_gnss.latitude = input_tag.pos_3d[1];
  // output_gnss.longitude = input_tag.pos_3d[0];
  // output_gnss.altitude = input_tag.pos_3d[2];

  // TODO: Add covariance into gnss topic
  // for (int i = 0; i < p_pose_cov.size(); i++)
  // {
  //   output_gnss.position_covariance[i] = p_pose_cov[i];
  // }
  // output_gnss.position_covariance_type = 1;

  // pub_gnss.publish(output_gnss);
}

bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ros::NodeHandle nh_local("~");
  nh_local.param<bool>("active", p_active, true);

  nh_local.param<std::string>("map_frame_id", p_map_frame_id, "map");

  nh_local.param<std::string>("framd_id", p_frame_id, "base_footprint");

  nh_local.param<double>("cutoff_freq", p_cutoff_freq, 5);

  lpf_gain = 1 - exp(-0.005 * 2 * M_PI * p_cutoff_freq);

  ROS_INFO("[UWB]: lpf gain set to: %lf", lpf_gain);

  p_linear_accel_cov.clear();
  p_linear_accel_cov.resize(9);
  nh_local.getParam("linear_acceleration_covariance", p_linear_accel_cov);

  p_angular_vel_cov.clear();
  p_angular_vel_cov.resize(9);
  nh_local.getParam("angular_velocity_covariance", p_angular_vel_cov);

  p_pose_cov.clear();
  p_pose_cov.resize(36);
  nh_local.getParam("pose_covariance", p_pose_cov);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nlink_tagframe_converter");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  ros::ServiceServer params_srv_;
  params_srv_ = nh_local.advertiseService("params", updateParams);

  std_srvs::Empty empt;
  updateParams(empt.request, empt.response);

  ros::Subscriber nlink_sub = nh.subscribe<nlink_parser::LinktrackTagframe0>("uwb_state", 20, &tagCallback);
  pub_imu = nh.advertise<sensor_msgs::Imu>("uwb_imu_raw", 20);
  pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("uwb_pose", 20);
  // pub_gnss = nh.advertise<sensor_msgs::NavSatFix>("uwb_gnss", 20);
  ros::spin();
}
