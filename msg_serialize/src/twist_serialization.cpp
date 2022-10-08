/**
 * 
 * @file twist_serialization.cpp
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
 * @date 2022-08-15
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
ros::Publisher string_pub;
void twist_callback(geometry_msgs::Twist::ConstPtr msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_ptr<uint8_t[]> buffer(new uint8_t[serial_size]);

  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);
  std_msgs::String s;
  s.data.assign(reinterpret_cast<char*>(buffer.get()), serial_size);
  string_pub.publish(s);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_serialization");
  ros::NodeHandle nh;
  ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &twist_callback);
  string_pub = nh.advertise<std_msgs::String>("serialized_msg", 10);
  ros::spin();
}
