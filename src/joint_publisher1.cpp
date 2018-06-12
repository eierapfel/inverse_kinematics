#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include "math.h"
#include <sstream>
#include "inverse_kinematics/destination_topic.h"


int main(int argc,char **argv)
{
  ros::init(argc, argv,"joint_publisher1");
  ros::NodeHandle nh;

  //ros::Subscriber joint_sub = nh.subscribe("destination_msg",50,msgCallback);
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);

  sensor_msgs::JointState jsm;
  jsm.name.resize(5);
  jsm.name[0] = "joint1";
  jsm.name[1] = "joint2";
  jsm.name[2] = "joint3";
  jsm.name[3] = "joint4";
  jsm.name[4] = "joint5";
  jsm.position.resize(5);
  ros::Rate rate(10);
  int count = 0;
  while (ros::ok())
   {
    jsm.header.stamp = ros::Time::now();
    jsm.position[0] = 0.2*count;
    jsm.position[1] = 0.2*count;
    jsm.position[2] = 0.2*count;
    jsm.position[3] = 0.2*count;
    jsm.position[4] = 0.2*count;
    count++;
    joint_pub.publish(jsm);
    rate.sleep();


  }
  return 0;
}
