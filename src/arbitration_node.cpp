#include <arbitration_utils/arbitration_utils.h>
#include <arbitration_utils/utils.h>
#include <ros/ros.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbitration_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ArbitrationUtils au(nh);
  
  au.addObj();
  
  ros::Duration(0.1).sleep();
  
  ros::Rate rate(10);
  
  while (ros::ok())
  {
  double man = au.getCurrentManipulability();
  ROS_INFO_STREAM_THROTTLE(2.0, CYAN << "manipulability: "<<man);
  
  double dis = au.checkWorldCollisionDistance();
  ROS_INFO_STREAM_THROTTLE(2.0, BLUE << "distance: "<<dis);
  
  au.publishAlpha();
  
  rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

