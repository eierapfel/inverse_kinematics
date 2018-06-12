#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include "math.h"
#include <sstream>
int main(int argc,char **argv)
{
  ros::init(argc, argv,"joint_publisher2");
  ros::NodeHandle nh;

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);

  sensor_msgs::JointState jsm;
  jsm.name.resize(3);
  jsm.name[0] = "joint1";
  jsm.name[1] = "joint2";
  jsm.name[2] = "joint3";

  jsm.position.resize(3);
  ros::Rate rate(10);
  int count = 0;
  while (ros::ok())
   {
    jsm.header.stamp = ros::Time::now();
    jsm.position[0] = 0.2*count;
    jsm.position[1] = 0.2*count;
    jsm.position[2] = 0.2*count;
    
    count++;
    joint_pub.publish(jsm);
    rate.sleep();


  }
  return 0;
}
