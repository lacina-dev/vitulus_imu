#include <stdio.h>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
ros::Publisher pub;


void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
//  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    sensor_msgs::Imu msg_enu;
    msg_enu.header = msg->header;

    tf2::Quaternion rotate_quat, nwu_quat, enu_quat;
    rotate_quat.setRPY( 0, 0, 1.5707 );  // 90 degrees rpy
    tf2::convert(msg->orientation , nwu_quat);
    enu_quat = rotate_quat*nwu_quat;  // Calculate the new orientation
    enu_quat.normalize();
//    tf2::convert(enu_quat, msg_enu.orientation);



    msg_enu.orientation.x = -enu_quat[0];
    msg_enu.orientation.y = -enu_quat[1];
    msg_enu.orientation.z = -enu_quat[2];
    msg_enu.orientation.w = -enu_quat[3];

    msg_enu.angular_velocity.x = msg->angular_velocity.x;
    msg_enu.angular_velocity.y = msg->angular_velocity.y;
    msg_enu.angular_velocity.z = msg->angular_velocity.z;

    msg_enu.linear_acceleration.x = msg->linear_acceleration.x;
    msg_enu.linear_acceleration.y = msg->linear_acceleration.y;
    msg_enu.linear_acceleration.z = msg->linear_acceleration.z;

  pub.publish(msg_enu);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nwu_to_enu");
  ros::NodeHandle n;
  ROS_INFO("Node nwu_to_enu started");

  pub = ros::Publisher(n.advertise<sensor_msgs::Imu>("enu1_out", 1));
  ros::Subscriber sub = n.subscribe("nwu1_in", 10, chatterCallback);
  ros::spin();

  return 0;
}

