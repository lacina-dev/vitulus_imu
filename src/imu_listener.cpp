#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
ros::Publisher pub;

double a = 1.001308;
double b = 0.0;
double c = 0.0;
double d = 0.038358;
double e = 0.0;
double f = 1.002202;
double g = 0.0;
double h = -0.118057;
double i = 0.0;
double j = 0.0;
double k = 0.999718;
double l = 0.075905;
double m = 0.0;
double n = 0.0;
double o = 0.0;
double p = 1.0;
//Eigen::Matrix4d C = {{a, b, c, d}, {e, f, g, h}, {i, j, k, l}, {m, n, o, p}};
//Eigen::Matrix4d C(4,4) ;
//C(0,0)=a;
//C(0,1)=b;
//C << a, b, c, d,
//     e, f, g, h,
//     i, j, k, l,
//     m, n, o, p;
//std::cout << C;

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);


//  imu_publisher->publish(msg);
//  Eigen::Matrix4d C {{1.001308, 0.0, 0.0, 0.038358}, {0.0, 1.002202, 0.0, -0.118057}, {0.0, 0.0, 0.999718, 0.075905}, {0.0, 0.0, 0.0, 1.0}};

//    Eigen::Matrix<double, 4, 4> C {{a, b, c, d}, {e, f, g, h}, {i, j, k, l}, {m, n, o, p}};
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;

  pub = ros::Publisher(n.advertise<sensor_msgs::Imu>("/bno085/imu_calibrated", 1));
  ros::Subscriber sub = n.subscribe("/bno085/imu", 10, chatterCallback);
 ros::spin();

  return 0;
}

