#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <object_loader_msgs/AddObjects.h>


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

  std::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group_;
  std::shared_ptr< planning_scene::PlanningScene > planning_scene_ ;
  robot_model::RobotModelPtr robot_model_; 
  robot_state::JointModelGroup* joint_model_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  
  Alpha(ros::NodeHandle nh) : nh_(nh)
  {
    if(!nh_.getParam("height",   height_   ))
      height_   = 0.99;
    if(!nh_.getParam("slope",    slope_    ))
      slope_    = 20;
    if(!nh_.getParam("midpoint", midpoint_ ))
      midpoint_ = 0.25;
    if(!nh_.getParam("offset",   offset_   ))
      offset_   = 0.01;


  std::string planning_group = "robot_1";  //TODO: better from params
  move_group_.reset(new moveit::planning_interface::MoveGroupInterface (planning_group));
  robot_model_ = robot_model_loader::RobotModelLoader ( "robot_description" ).getModel();
  joint_model_group_ = robot_model_->getJointModelGroup(planning_group);
  planning_scene_.reset (new planning_scene::PlanningScene( robot_model_ ));
  
  robot_model_loader_.reset( new robot_model_loader::RobotModelLoader ( "robot_description" ) );
  
  planning_scene_monitor_.reset( new planning_scene_monitor::PlanningSceneMonitor( robot_model_loader_ ) ); 
  planning_scene_monitor_->startSceneMonitor( );
  planning_scene_monitor_->startWorldGeometryMonitor( );
  planning_scene_monitor_->startStateMonitor( );

  };
  
  double computeAlpha(double x) 
  {
    return height_ / (1 + exp(-slope_ * (x - midpoint_))) + offset_;
  }
  
  double getDistanceFrom(const std::string& base, const std::string& target)
  {
    distance_ = -1.0;
    try
    {
      if(!listener_.waitForTransform(base, target, ros::Time::now(), ros::Duration(1.0)))
        return distance_;
      
      listener_.lookupTransform (base, target, ros::Time(0)    , transform_);
      
      tf::poseTFToMsg(transform_, distance_from_target_);
      
      tf::vectorTFToEigen(transform_.getOrigin(),dist_xyz_);
      distance_ = dist_xyz_.norm();
      return distance_;
    }
    catch (tf::TransformException &ex) 
    {
      ROS_WARN_STREAM_THROTTLE(10.0,"[arbitration_node] transform between "<< base << " and " << target << " does not yet exists . waiting for it");
      ROS_ERROR("%s",ex.what());
    }
    return -1.0;
  }
  
  void publishDistance()
  {
    pub_distance.publish(distance_from_target_);
  }

void getPlanningScene   ( ros::NodeHandle& nh, planning_scene::PlanningScenePtr& ret )
{
  ros::ServiceClient planning_scene_service;
  planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene> ( "get_planning_scene" );
  if ( !planning_scene_service.waitForExistence ( ros::Duration ( 5.0 ) ) )
  {
      ROS_ERROR ( "getPlanningScene Failed: service '%s/get_planning_scene' does not exist", nh.getNamespace().c_str() );
  }

  moveit_msgs::PlanningScene planning_scene_msgs;

  {
    /// ROBOT_STATE
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.ROBOT_STATE;
    if ( !planning_scene_service.call ( request, response ) )
    {
      ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    
    planning_scene_msgs.name = response.scene.name;
    planning_scene_msgs.robot_state = response.scene.robot_state;
  }
  {
    // WORLD_OBJECT_GEOMETRY && WORLD_OBJECT_NAMES
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.WORLD_OBJECT_GEOMETRY;
    if ( !planning_scene_service.call ( request, response ) )
    {
      ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    planning_scene_msgs.world.collision_objects = response.scene.world.collision_objects;
  }
  {
    // OCTOMAP
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.OCTOMAP;
    if ( !planning_scene_service.call ( request, response ) )
    {
      ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    planning_scene_msgs.world.octomap = response.scene.world.octomap;
  }
  {
    // TRANSFORMS
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.TRANSFORMS;
    if ( !planning_scene_service.call ( request, response ) )
    {
      ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    planning_scene_msgs.fixed_frame_transforms = response.scene.fixed_frame_transforms;
  }
  {
    // ALLOWED_COLLISION_MATRIX
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.ALLOWED_COLLISION_MATRIX;
    if ( !planning_scene_service.call ( request, response ) )
    {
      ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    planning_scene_msgs.allowed_collision_matrix = response.scene.allowed_collision_matrix;
  }
  {
    // LINK_PADDING_AND_SCALING
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.LINK_PADDING_AND_SCALING;
    if ( !planning_scene_service.call ( request, response ) )
    {
      ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    planning_scene_msgs.link_padding = response.scene.link_padding;
    planning_scene_msgs.link_scale   = response.scene.link_scale;
  }
  {
    // OBJECT_COLORS
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.LINK_PADDING_AND_SCALING;
    if ( !planning_scene_service.call ( request, response ) )
    {
        ROS_WARN ( "Could not call planning scene service to get object names" );
    }
    planning_scene_msgs.object_colors = response.scene.object_colors;
  }

  ret->setPlanningSceneMsg ( planning_scene_msgs );

  return;
}

  double checkWorldCollisionDistance()
  {  
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_scene_monitor::LockedPlanningSceneRW( planning_scene_monitor_ ); 
    robot_state::RobotState robot_state = ps->getCurrentStateNonConst();
    collision_detection::AllowedCollisionMatrix *acm = &ps->getAllowedCollisionMatrixNonConst();
    
    getPlanningScene ( nh_, planning_scene_ );
    
    double dist = planning_scene_->distanceToCollision(robot_state, *acm);
    
    if (dist <= 0.0001)
    {
      ROS_INFO_STREAM(RED<<"careful ! robot in collision");
      dist = 0.0001;
    }

    return dist;
  }






private:
  tf::TransformListener listener_;
  tf::StampedTransform transform_;
  double height_, slope_, midpoint_, offset_;
  ros::NodeHandle nh_;
  double distance_;
  Eigen::Vector3d dist_xyz_;
  ros::Publisher pub_distance = nh_.advertise<geometry_msgs::Pose>("/distance_from_target", 10);  
  geometry_msgs::Pose distance_from_target_;
};


void vecToPose(const std::vector<double>& pose ,geometry_msgs::Pose& gpose)
  {
    tf::Pose transform;
    
    tf::Vector3 v = tf::Vector3(pose.at(0),pose.at(1),pose.at(2));
    tf::Quaternion q;
    
    if(pose.size() == 6)
    q = tf::createQuaternionFromRPY(pose.at(3),pose.at(4),pose.at(5)); 
    else
    q = tf::Quaternion(pose.at(4),pose.at(5),pose.at(6),pose.at(3)); 
    
    transform.setOrigin(v);
    transform.setRotation(q); 
    
    tf::poseTFToMsg(transform, gpose);
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbitration_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();


  ros::ServiceClient add_obj_ = nh.serviceClient<object_loader_msgs::AddObjects> ( "add_object_to_scene" ) ;
  ROS_INFO_STREAM("waiting for service: "<< add_obj_.getService());
  add_obj_.waitForExistence();

  ROS_DEBUG_STREAM("adding object");
  


  //TODO::make a function to add objects
{
  geometry_msgs::Pose obj_pose;
  std::vector<double> pos;
  if ( !nh.getParam ( "object_geometries/pose1", pos) )
  {
    ROS_ERROR_STREAM(nh.getNamespace() << " /object_geometries/pose not set. return");
    return -1;
  }
  vecToPose(pos,obj_pose);
  object_loader_msgs::AddObjects srv;  
  {
    object_loader_msgs::Object obj;
    
    obj.object_type="wall";
    
    obj.pose.pose = obj_pose;
    obj.pose.header.frame_id = "base_link";
    srv.request.objects.push_back(obj);
  }
  add_obj_.call(srv);
}
{
  geometry_msgs::Pose obj_pose;
  std::vector<double> pos;
  if ( !nh.getParam ( "object_geometries/pose2", pos) )
  {
    ROS_ERROR_STREAM(nh.getNamespace() << " /object_geometries/pose not set. return");
    return -1;
  }
  vecToPose(pos,obj_pose);
  object_loader_msgs::AddObjects srv;  
  {
    object_loader_msgs::Object obj;
    
    obj.object_type="wall";
    
    obj.pose.pose = obj_pose;
    obj.pose.header.frame_id = "base_link";
    srv.request.objects.push_back(obj);
  }
  add_obj_.call(srv);
}
  

  
  ros::Rate rate(125);
  
  ros::Publisher pub_alpha = nh.advertise<std_msgs::Float32>("/alpha", 10);  

  double trg_distance  ;
  double coll_distance  ;
  double alpha ;
  std_msgs::Float32 alp;
  
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
  
  
  ros::Publisher pub_col = nh.advertise<std_msgs::Float32>("/distance_from_col", 10);  
  ros::Publisher pub_trg = nh.advertise<std_msgs::Float32>("/distance_from_trg", 10);  
  
  std_msgs::Float32 d_trg_msg;
  std_msgs::Float32 d_col_msg;

  Alpha al(nh);

  while (ros::ok())
  {
    trg_distance  = al.getDistanceFrom(ee_pose,target_pose);
    // trg_distance  = 1.0;
    coll_distance = al.checkWorldCollisionDistance();
    // coll_distance = 1.0;
    
    if(trg_distance<0.0)
    {
      ROS_WARN_STREAM_THROTTLE(10.0,"[arbitration_node] waiting for transform from " << target_pose << " and " << ee_pose );
      continue;
    }
    
    double alpha_trg = al.computeAlpha(trg_distance);
    double alpha_col = al.computeAlpha(coll_distance);

    alpha = std::min(alpha_col,alpha_trg);

    alp.data = alpha;
    pub_alpha.publish(alp);

    d_trg_msg.data = trg_distance;
    d_col_msg.data = coll_distance;

    pub_trg.publish(d_trg_msg);
    pub_col.publish(d_col_msg);

    ROS_INFO_STREAM_THROTTLE(.1,GREEN<< "[arbitration_node]  collision distance: " << coll_distance<<" --> alpha: "<<alpha_col);
    ROS_INFO_STREAM_THROTTLE(.1,CYAN<<"[arbitration_node] target distance : "<< trg_distance <<" --> alpha: "<<alpha_trg);
    ROS_INFO_STREAM_THROTTLE(.1,MAGENTA<<"[arbitration_node] alpha : "<< alpha);

    al.publishDistance();

    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

