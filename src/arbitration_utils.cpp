#include <arbitration_utils/arbitration_utils.h>
#include <arbitration_utils/utils.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

ArbitrationUtils::ArbitrationUtils(ros::NodeHandle nh)
:nh_(nh)
{
  std::string planning_group;
  if ( !nh_.getParam ( "planning_group", planning_group) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /planning_group not set. return");
    return;
  }
  if ( !nh_.getParam ( "base_link", base_link_) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /base_link not set. return");
    return;
  }
  std::string tool_link;
  if ( !nh_.getParam ( "tool_link", tool_link) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /tool_link not set. return");
    return;
  }
  std::string ee_link;
  if ( !nh_.getParam ( "ee_link", ee_link) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /ee_link not set. return");
    return;
  }
  std::vector<std::string> joint_names;
  if ( !nh_.getParam ( "joint_names", joint_names) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /joint_names not set. return");
    return;
  }
  std::vector<double> pos;
  if ( !nh_.getParam ( "object_geometries/pose", pos) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /object_geometries/pose not set. return");
    return;
  }
  vecToPose(pos,obj_pose_);
  
  if ( !nh_.getParam ( "max_fl", max_fl_) )
  {
    ROS_WARN_STREAM (nh_.getNamespace() << " /max_fl not set. default 0.99");
    max_fl_ = 0.99;
  }
  if ( !nh_.getParam ( "min_fl", min_fl_) )
  {
    ROS_WARN_STREAM (nh_.getNamespace() << " /min_fl not set. default 0.01");
    min_fl_ = 0.01;
  }
  
  
  move_group_.reset(new moveit::planning_interface::MoveGroupInterface (planning_group));
  robot_model_ = robot_model_loader::RobotModelLoader ( "robot_description" ).getModel();
  joint_model_group_ = robot_model_->getJointModelGroup("manipulator");
  planning_scene_.reset (new planning_scene::PlanningScene( robot_model_ ));
  
  robot_model_loader_.reset( new robot_model_loader::RobotModelLoader ( "robot_description" ) );
  
  planning_scene_monitor_.reset( new planning_scene_monitor::PlanningSceneMonitor( robot_model_loader_ ) ); 
  planning_scene_monitor_->startSceneMonitor( );
  planning_scene_monitor_->startWorldGeometryMonitor( );
  planning_scene_monitor_->startStateMonitor( );
  
  add_obj_ = nh_.serviceClient<object_loader_msgs::AddObjects> ( "add_object_to_scene" ) ;
  ROS_INFO_STREAM("waiting for service: "<< add_obj_.getService());
  add_obj_.waitForExistence();
  
  std::string alpha_topic;
  if ( !nh_.getParam ( "alpha_topic", alpha_topic) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /alpha_topic not set. default "<< nh_.getNamespace()<<"/alpha");
    alpha_topic = nh_.getNamespace()+"/alpha";
  }
  alpha_pub_ = nh_.advertise< std_msgs::Float32 >(alpha_topic, 1000);
  
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("Urdf robot_description '%s' does not exist",(nh_.getNamespace()+"/robot_description").c_str());
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  chain_bt_  = rosdyn::createChain(urdf_model, base_link_, tool_link, gravity);
  chain_bee_ = rosdyn::createChain(urdf_model, base_link_, "tool0", gravity);
  
  for (auto j:joint_names)
  {
    joint_limits_interface::JointLimits limits;
    urdf::JointConstSharedPtr urdf_joint = urdf_model.getJoint(j);
    if( ! getJointLimits(urdf_joint, limits) )
    {
      ROS_ERROR_STREAM("error in reading joint "<< j <<" limits");
    }
    upper_bounds_.push_back ( limits.max_position );
    lower_bounds_.push_back ( limits.min_position );
  }
  
  std::vector<double> cj = move_group_->getCurrentJointValues();
  
  for (size_t i=0;i<joint_names.size();i++)
    ROS_INFO_STREAM(BLUE<<"joint_"<< joint_names.at(i) <<" current:" << cj.at(i) <<" - upper joint: "<< upper_bounds_.at(i) <<" - lower joint: "<<lower_bounds_.at(i));
  
  
  std::string fis;
  if ( !nh_.getParam ( "alpha_fis", fis) )
  {
    ROS_ERROR_STREAM (nh_.getNamespace() << " /alpha_fis not set. return");
    return;
  }
  std::string path = ros::package::getPath("arbitration_utils");
  path += "/config/";
  path += fis;
  
  ROS_INFO_STREAM("recovering path: " << path);
  
  engine_ = fl::FisImporter().fromFile(path);
  std::string status;
  if (not engine_->isReady(&status))
    ROS_ERROR_STREAM("engine is not ready");

  manipulability_  = engine_->getInputVariable("manipulability");
  distance_        = engine_->getInputVariable("distance");
  reach_           = engine_->getInputVariable("reach");
  endpoint_        = engine_->getInputVariable("endpoint");
  alpha_           = engine_->getOutputVariable("alpha");
  
}


double ArbitrationUtils::getManipulability(const std::vector<double> joints)
{
  Eigen::VectorXd j;
  
  j.resize(joints.size());
  for (int i=0; i<joints.size(); i++)
    j(i) = joints[i];
  
  Eigen::MatrixXd  J_b = chain_bt_->getJacobian (j);
  
  std::vector<double> prod;
  
  for (auto j:joints)
    ROS_DEBUG_STREAM("j: "<<j);
  
  for (int i=0; i<joints.size(); i++)
  {
    double num = (joints[i] - lower_bounds_[i])*(upper_bounds_[i] - joints[i]); 
    double den = upper_bounds_[i] - lower_bounds_[i];
    prod.push_back( num / pow(den,2.0));
  }
  double min_prod = *std::min_element(prod.begin(),prod.end());
  double k = 100;
  double penalty = 1 - exp(-k * min_prod);
  
  ROS_DEBUG_STREAM("mu: "<<(std::sqrt((J_b * J_b.transpose()).determinant())));
  ROS_DEBUG_STREAM("penal: "<<penalty);
  
  return (std::sqrt((J_b * J_b.transpose()).determinant())) * penalty;
}


double ArbitrationUtils::getCurrentManipulability()
{
  return getManipulability(move_group_->getCurrentJointValues());
}


double ArbitrationUtils::getReach()
{
  std::vector<double> cjv = move_group_->getCurrentJointValues();
  Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(cjv.data(), cjv.size());
  Eigen::Affine3d T_be = chain_bee_->getTransformation(vec);
  
  double reach = T_be.translation().norm();
  
  return reach; 
}

double ArbitrationUtils::getDistanceFrom(const std::string& base, const std::string& target)
{
  tf::StampedTransform transform;
  tf::StampedTransform tt;
  double distance = 0.5;
  try
  {
    listener_.waitForTransform(base, target, ros::Time::now(), ros::Duration(5.0));
    listener_.lookupTransform (base, target, ros::Time(0)    , transform);
    listener_.lookupTransform (base, "TBT", ros::Time(0)    , tt);
    Eigen::Vector3d v;
    
    tf::vectorTFToEigen(transform.getOrigin(),v);
    distance = v.norm();
//     ROS_INFO_STREAM_THROTTLE(1.0,"distance between TF: " << distance);
    
    Eigen::Vector3d vt;
    tf::vectorTFToEigen(tt.getOrigin(),vt);
    double tttpdist = vt.norm();
    if (tttpdist < 0.001)
      ROS_INFO_STREAM_THROTTLE(1,GREEN<<"distance between tip tbt ok <1mm: " << tttpdist );
    else
      ROS_INFO_STREAM_THROTTLE(1,RED<<"distance between tip tbt > 1mm: : " << tttpdist );
    
    
    ros::Publisher pub = nh_.advertise<std_msgs::Float32>("/distance_to_goal", 10);
    
    std_msgs::Float32 msg;
    msg.data = distance;
    pub.publish(msg);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_WARN_STREAM_THROTTLE(2.0,"transform not yet recovered, skipping");
    ROS_ERROR("%s",ex.what());
  }
  
  
  return distance;
  
}


void ArbitrationUtils::getPlanningScene   ( ros::NodeHandle& nh, planning_scene::PlanningScenePtr& ret )
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


void ArbitrationUtils::addObj()
{
  ROS_DEBUG_STREAM("adding object");
  
  object_loader_msgs::AddObjects srv;  
  {
    object_loader_msgs::Object obj;
    
    obj.object_type="box";
    
    obj.pose.pose = obj_pose_;
    obj.pose.header.frame_id = base_link_;
    srv.request.objects.push_back(obj);
  }
  
  add_obj_.call(srv);
  
}


double ArbitrationUtils::checkWorldCollisionDistance()
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



double ArbitrationUtils::computeAlpha(const double& dist, const double& reach, const double& man, const double& clos)
{
  manipulability_->setValue(man);
  distance_->setValue(dist);
  reach_->setValue(reach);
  endpoint_->setValue(clos);
  
  engine_->process();
  
  double out = alpha_->getValue();
  
  if ( isnan(out) )
  {
    ROS_INFO_STREAM_THROTTLE(5.0,"setting alpha to 0.99");
    out = max_fl_;
  }
  
  double ret = (out - min_fl_)/(max_fl_ - min_fl_);
  
  ROS_DEBUG_STREAM_THROTTLE(2.0,"given manipulability: "<<manipulability_->getValue()<<", and reach "<< reach_->getValue()<<", and distance "<< distance_->getValue()<<", and endpoint closeness "<< endpoint_->getValue()<<", fl returns alpha = "<<ret);
  
  return ret;
}


void ArbitrationUtils::publishAlpha(const double& alpha)
{  
  std_msgs::Float32 alpha_msg;
  
  alpha_msg.data = alpha;
  
  alpha_pub_.publish(alpha_msg);
  
  return ;
}

void ArbitrationUtils::vecToPose(const std::vector<double>& pose ,geometry_msgs::Pose& gpose)
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








