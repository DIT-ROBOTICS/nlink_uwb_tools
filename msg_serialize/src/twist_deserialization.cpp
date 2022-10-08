/**
 *
 * @file twist_deserialization.cpp
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
#include <nlink_parser/LinktrackNodeframe0.h>

ros::Publisher twist_pub;

void node_frame_callback(nlink_parser::LinktrackNodeframe0::ConstPtr msg)
{
  geometry_msgs::Twist twist;
  uint32_t serial_size = ros::serialization::serializationLength(twist);
  boost::shared_ptr<uint8_t[]> buffer(new uint8_t[serial_size]);
  if (msg->nodes.size())
  {
    for (int i = 0; i < serial_size; i++)
    {
      buffer[i] = msg->nodes[0].data[i];
    }
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  ros::serialization::Serializer<geometry_msgs::Twist>::read(stream, twist);

  twist_pub.publish(twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_deserialization");
  ros::NodeHandle nh;
  ros::Subscriber twist_sub = nh.subscribe<nlink_parser::LinktrackNodeframe0>("node_frame", 10, &node_frame_callback);
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::spin();
}
