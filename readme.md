# nlink_uwb_tools

This repo is a collection of ros packages for LinkTrack UWB, containing the main Linktrack package, dependencies and message serialize tool.

## Install

### Clone

Under `catkin_ws/src`

`git clone --recursive git@github.com:DIT-ROBOTICS/nlink_uwb_tools.git`

### Install

Under `catkin_ws/src`

`cd nlink_uwb_tools/serial`

Under `catkin_ws/src/nlink_uwb_tools/serial`

`make; sudo make install`

`cd ../../..`

Under `catkin_ws`

`catkin_make`

Note: `catkin_make` is recommended, `catkin build` or `catkin_make_isolated` is NOT tested well.

### ttyUSB permission

Check device connected

`ll /dev | grep ttyUSB`

If I/O issue, make sure USB permission is r/w-able.

try 

`sudo usermod -a -G dialout $USER`

Add the user into dialout group to get permission permanently on most hosts.

## Run

Default `linktrack.0` is working on ttyUSB0, and run the serial node and UWB node.

Default `linktrack.1` is working on ttyUSB1, and run the DEserial node and UWB node.

`roslaunch msg_serialize linktrack.0.launch`

`roslaunch msg_serialize linktrack.1.launch`

![bobobob](/rosgraph.png)

## Nodes

1. twist_serialization

Convert twist to a serialized string, publish to UWB and transmit.

   1. Subscribe Topic

      cmd_vel (geometry_msgs/Twist)

   2. Publish Topic

      serialized_msg (std_msgs/String)

2. twist_deserialization

Receive a serialized string from UWB, and convert the serialized string back to the twist.

   1. Subscribe Topic

      node_frame (nlink_parser/LinktrackNodeframe0)

   2. Publish Topic

      cmd_vel (geometry_msgs/Twist)
