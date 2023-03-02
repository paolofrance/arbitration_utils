#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>



static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";



class Alpha
{
public:
  
  Alpha(double h, double s, double m, double o)
        : height_(h)
        , slope_(s) 
        , midpoint_(m) 
        , offset_(o) {};
  double computeAlpha(double x) 
  {
    return height_ / (1 + exp(-slope_ * (x - midpoint_))) + offset_;
  }
  
  double getDistanceFrom(const std::string& base, const std::string& target)
  {
    double distance = -1.0;
    try
    {
      if(!listener_.waitForTransform(base, target, ros::Time::now(), ros::Duration(1.0)))
        return distance;
      
      listener_.lookupTransform (base, target, ros::Time(0)    , transform_);
      Eigen::Vector3d v;
      
      tf::vectorTFToEigen(transform_.getOrigin(),v);
      distance = v.norm();
      return distance;
    }
    catch (tf::TransformException &ex) 
    {
      ROS_WARN_STREAM_THROTTLE(10.0,"[arbitration_node] transform between "<< base << " and " << target << " does not yet exists . waiting for it");
      ROS_ERROR("%s",ex.what());
    }
    return -1.0;
  }

private:
  tf::TransformListener listener_;
  tf::StampedTransform transform_;
  double height_, slope_, midpoint_, offset_;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbitration_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  ros::Rate rate(125);
  
  ros::Publisher pub = nh.advertise<std_msgs::Float32>("/alpha", 10);  

  double clos  ;
  double alpha ;
  std_msgs::Float32 alp;
  
  double height, slope, midpoint, offset;
  
  if(!nh.getParam("height",   height   ))
    height   = 0.99;
  if(!nh.getParam("slope",    slope    ))
    slope    = 20;
  if(!nh.getParam("midpoint", midpoint ))
    midpoint = 0.25;
  if(!nh.getParam("offset",   offset   ))
    offset   = 0.01;
  
  std::string target_pose, ee_pose;
  if(!nh.getParam("target_pose", target_pose))
  {
    target_pose = "target_pose";
    ROS_WARN_STREAM("param /target_pose not set . DEFAULT : " << target_pose );
  }
  if(!nh.getParam("ee_pose",   ee_pose))
  {
    target_pose = "ee_pose";
    ROS_WARN_STREAM("param /ee_pose not set . DEFAULT : " << ee_pose );
  }
  
  
  Alpha al(height, slope, midpoint, offset);
  
  while (ros::ok())
  {
    clos  = al.getDistanceFrom(ee_pose,target_pose);
    
    if(clos<0.0)
    {
      ROS_WARN_STREAM_THROTTLE(10.0,"[arbitration_node] waiting for transform from " << target_pose << " and " << ee_pose );
      continue;
    }
    
    alpha = al.computeAlpha(clos);
    alp.data = alpha;
    pub.publish(alp);
    ROS_INFO_STREAM_THROTTLE(1.0," [arbitration_node] distance : "<< clos <<" --> alpha: "<<alpha);
    
    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

