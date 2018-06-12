#include "ros/ros.h"
#include "inverse_kinematics/destination_topic.h"


void msgCallback(const inverse_kinematics::destination_topic::ConstPtr& msg)
{
  ROS_INFO("now: x = %f", msg->x);
  ROS_INFO("now: y = %f", msg->y);
  ROS_INFO("now: z = %f", msg->z);
  ROS_INFO("ang1 = %f", msg->ang1);
  ROS_INFO("ang2 = %f", msg->ang2);
  ROS_INFO("ang3 = %f", msg->ang3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber inverse_sub = nh.subscribe("destination_msg",50,msgCallback);

  ros::spin();
  return 0;
}
