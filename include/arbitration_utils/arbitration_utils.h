#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <object_loader_msgs/AddObjects.h>

#include <rosdyn_core/primitives.h>

#include <fl/Headers.h>

class ArbitrationUtils
{
public:
  ros::NodeHandle nh_;
  std::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group_;
  std::shared_ptr< planning_scene::PlanningScene > planning_scene_ ;
  robot_model::RobotModelPtr robot_model_; 
  robot_state::JointModelGroup* joint_model_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  
  std::string base_link_;
  
  rosdyn::ChainPtr chain_bt_;
  
  std::vector<double> lower_bounds_; 
  std::vector<double> upper_bounds_ ;
  
  ros::ServiceClient  add_obj_ ;
  ros::Publisher alpha_pub_;
  
  double max_fl_;
  double min_fl_;

  
  fl::Engine*         engine_;
  fl::InputVariable*  manipulability_ ;
  fl::InputVariable*  distance_ ;
  fl::OutputVariable* alpha_ ;
  
  ArbitrationUtils(ros::NodeHandle nh);
  double getManipulability(const std::vector<double> joints);
  double getCurrentManipulability();
  void getPlanningScene   ( ros::NodeHandle& nh
                        , planning_scene::PlanningScenePtr& ret );
  void addObj();
  double checkWorldCollisionDistance();
  double computeAlpha(const double& dist, const double& man);
  void   publishAlpha();

};
