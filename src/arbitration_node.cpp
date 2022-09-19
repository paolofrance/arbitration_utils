#include <arbitration_utils/arbitration_utils.h>
#include <arbitration_utils/utils.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbitration_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ArbitrationUtils au(nh);
  
  au.addObj();
  
  ros::Duration(0.1).sleep();
  
  ros::Rate rate(125);
  
  ros::Publisher pubd = nh.advertise<std_msgs::Float32>("/dist", 1000);
  ros::Publisher pubr = nh.advertise<std_msgs::Float32>("/reach", 1000);
  ros::Publisher pubm = nh.advertise<std_msgs::Float32>("/manip", 1000);
  ros::Publisher puba = nh.advertise<std_msgs::Float32>("/alp", 1000);
  ros::Publisher pubc = nh.advertise<std_msgs::Float32>("/closeness", 1000);
  
  while (ros::ok())
  {
    double man = au.getCurrentManipulability();
    double reach = au.getReach();
    double dis = au.checkWorldCollisionDistance();
    double clos = au.getDistanceFrom("target_pose");
    
    double alpha = au.computeAlpha(.25, reach, man, clos);
    ROS_INFO_STREAM_THROTTLE(2.0, CYAN << "manipulability: "<<man << BLUE << ", reach: "<<reach<< GREEN << ", distance: "<<dis << YELLOW << ", closeness: "<<clos << MAGENTA << " ---> alpha: "<<alpha);
    
//     au.publishAlpha(alpha);
    
    {
      std_msgs::Float32 m;
      m.data = dis;
//       pubd.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = reach;
//       pubr.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = man;
//       pubm.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = alpha;
//       puba.publish(m);
    }
    {
      std_msgs::Float32 m;
      m.data = clos;
      pubc.publish(m);
    }
    
    
    
    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

