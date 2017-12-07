#include "skill_transfer/conversions.h"
#include "skill_transfer/twist_log.h"
#include "skill_transfer/task.h"
#include "skill_transfer/experiment.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <skill_transfer/MoveArmAction.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_listener.h>
#include <skill_transfer/FindEdgeAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>

class TaskExecutive
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<skill_transfer::MoveArmAction> ac_;
  ros::Subscriber link_state_sub_;
  ros::Subscriber set_link_state_sub_;
  ros::Subscriber tool_contact_sensor_state_sub_;
  bool running_ = false;
  TwistLog velocity_log_;
  TwistLog command_log_;
  double goal_distance_;
  std::string task_directory_path_;
  Task task_;
  std::string experiment_file_path_;
  Experiment experiment_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  actionlib::SimpleActionClient<skill_transfer::FindEdgeAction> find_edge_ac_;
  geometry_msgs::Point edge_point_;
  YAML::Node edge_point_node_;

public:
  TaskExecutive() : nh_("~"),
                    ac_("move_arm", true),
                    velocity_log_(10),
                    command_log_(10),
                    tfListener(tfBuffer),
                    find_edge_ac_("find_edge", true)
  {
    // Get task specifications
    if ( !nh_.getParam("task_directory", task_directory_path_) )
    {
      throw std::runtime_error("Could not find parameter 'task_directory' in namespace '" + nh_.getNamespace() + "'.");
    }
    
    std::string motion_directory_path;
    // Get task specifications
    if ( !nh_.getParam("motion_directory", motion_directory_path) )
    {
      throw std::runtime_error("Could not find parameter 'motion_directory' in namespace '" + nh_.getNamespace() + "'.");
    }
    
    std::string motiom_template_file_path;
    if ( !nh_.getParam("motiom_template_file_path", motiom_template_file_path) )
    {
      throw std::runtime_error("Could not find parameter 'motiom_template_file' in namespace '" + nh_.getNamespace() + "'.");
    }
    
    if ( !nh_.getParam("experiment_file_path", experiment_file_path_) )
    {
      throw std::runtime_error("Could not find parameter 'experiment_file_path' in namespace '" + nh_.getNamespace() + "'.");
    }
    
    ROS_INFO_STREAM("Loading experiment: " << experiment_file_path_);
    experiment_.load(experiment_file_path_);
    
    const auto task_file_path = task_directory_path_ + experiment_.task_file_name;
    ROS_INFO_STREAM("Loading task: " << task_file_path);
    task_.motion_directory_path = motion_directory_path;
    task_.load(task_file_path, motiom_template_file_path);
    
    ROS_INFO_STREAM("Task: " << task_.name);
    ROS_INFO_STREAM("Phases: " << task_.phases.size());
  
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");
    
    ROS_INFO("Waiting for edge action server to start.");
    find_edge_ac_.waitForServer();
    ROS_INFO("Edge action server started.");

    link_state_sub_ = nh_.subscribe("/ee_twist", 1,
                                    &TaskExecutive::onEeTwistMsg, this);
    set_link_state_sub_ = nh_.subscribe("/set_l_ee_twist", 1,
                                        &TaskExecutive::onSetEeTwistMsg, this);
                                        
    tool_contact_sensor_state_sub_ = nh_.subscribe("/tool_contact_sensor_state", 1,
                                                   &TaskExecutive::onToolContactSensorStateMsg, this);                           
  }

  ~TaskExecutive()
  {
  }

  void start()
  {
    findTargetObjectEdgePoint();
  
    sendNextGoal();
  }

  void onFinish(const actionlib::SimpleClientGoalState &state,
                const skill_transfer::MoveArmResultConstPtr &result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ros::shutdown();
  }

  void onFeedback(const skill_transfer::MoveArmFeedbackConstPtr &feedback)
  {
    goal_distance_ = feedback->distance;
  }

  void onEeTwistMsg(const geometry_msgs::TwistConstPtr &msg)
  {
    if (!running_)
      return;

    // Save twist to log
    velocity_log_.push(*msg);

    checkMeasuredVelocityStop();
  }

  void onSetEeTwistMsg(const geometry_msgs::TwistConstPtr &msg)
  {
    // Do not track velocities until the motion starts
    if (!running_)
      return;

    // Save twist to log
    command_log_.push(*msg);

    checkDesiredVelocityStop();
  }
  
  void onToolContactSensorStateMsg(const gazebo_msgs::ContactsStatePtr
   &msg)
  {
    // Do not track contact until the motion starts
    if (!running_)
      return;
      
    // Continue only when there's a contact
    if (msg->states.size() == 0) 
      return;
    
    checkContactStop();
  }

protected:

  void sendNextGoal()
  {
    velocity_log_.clear();
    command_log_.clear();
    
    const auto &spec = prepareMotionPhaseSpec();

    // Create and send goal
    skill_transfer::MoveArmGoal goal;
    goal.constraints = spec;

    ROS_INFO("Sending new goal.");
//    ROS_INFO_STREAM(spec);

    ac_.sendGoal(goal,
                 boost::bind(&TaskExecutive::onFinish, this, _1, _2),
                 actionlib::SimpleActionClient<skill_transfer::MoveArmAction>::SimpleActiveCallback(),
                 boost::bind(&TaskExecutive::onFeedback, this, _1));

    running_ = true;
  }

  void cancelCurrentGoal()
  {
    ac_.cancelGoal();
    running_ = false;
  }

// Stop Conditions

  void checkDesiredVelocityStop()
  {
    const auto &phase = task_.getCurrentPhase();
  
    if (goal_distance_ > phase.stop_activation_distance) 
      return;
    
    if (!command_log_.allFilledAndBelowThreshold(phase.stop_condition.desired_velocity_min))
      return;
      
    ROS_INFO("Desired Velocity Stop");

    completeStage();
  }
  
  void checkMeasuredVelocityStop()
  {
    const auto &phase = task_.getCurrentPhase();
  
    if (goal_distance_ > phase.stop_activation_distance) 
      return;
    
    if (!velocity_log_.allFilledAndBelowThreshold(phase.stop_condition.measured_velocity_min))
      return;
      
    ROS_INFO("Measured Velocity Stop");

    completeStage();
  }
  
  void checkContactStop()
  {
    const auto &phase = task_.getCurrentPhase();
      
    if ( !phase.stop_condition.contact )
      return;
      
    if (goal_distance_ > phase.stop_activation_distance) 
      return;
      
    ROS_INFO_STREAM("Contact Stop");
    
    completeStage();
  }
  
  void completeStage()
  {
    running_ = false;
    goal_distance_ = 1000000.0;
    
    // Increments task's internal index
    task_.completeCurrentPhase();
  
    if (task_.hasNextPhase())
    {
      ROS_INFO("Next");
      sendNextGoal();
    }
    else
    {
      ROS_INFO("Stop");
      cancelCurrentGoal();
    }
  }
  
  geometry_msgs::Point getTargetObjectReferencePoint()
  {
    addGraspTransforms();
    
    // Lookup TF
    geometry_msgs::TransformStamped transform_stamped;
    
    try 
    {
      transform_stamped = tfBuffer.lookupTransform(
        "target_object_frame", "tool_frame", ros::Time(0));  
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::shutdown();
    }
    
    // Get the edge point
    geometry_msgs::Point point;
    
    point.x = transform_stamped.transform.translation.x;
    point.y = transform_stamped.transform.translation.y;
    point.z = transform_stamped.transform.translation.z;
    
    return point;
  }
  
  void addGraspTransforms()
  {
    // Add gripper tranforms
    {
      geometry_msgs::TransformStamped transform_stamped;
    
      transform_stamped.header.frame_id = "r_gripper_tool_frame";
      transform_stamped.child_frame_id = "target_object_frame";
      transform_stamped.header.stamp = ros::Time::now();
      
      geometry_msgs::Transform transform;
      
      tf2::convert(experiment_.target_object_grasp_transform, transform);
      
      transform_stamped.transform = transform;
      
      tfBuffer.setTransform(transform_stamped, "task_executive", true);
    }
    
    {
      geometry_msgs::TransformStamped transform_stamped;
    
      transform_stamped.header.frame_id = "l_gripper_tool_frame";
      transform_stamped.child_frame_id = "tool_frame";
      transform_stamped.header.stamp = ros::Time::now();
      
      geometry_msgs::Transform transform;
      
      tf2::convert(experiment_.tool_grasp_transform, transform);
      
      transform_stamped.transform = transform;
      
      tfBuffer.setTransform(transform_stamped, "task_executive", true);
    }
  }
  
  void findTargetObjectEdgePoint() {
    const auto &point = getTargetObjectReferencePoint();
    
    skill_transfer::FindEdgeGoal goal;
    goal.point_cloud_file_name = experiment_.target_object_file_name;
    goal.reference_point = point;

    ROS_INFO("Finding edge point");

    find_edge_ac_.sendGoal(goal);
    
    //wait for the action to return
    bool finished_before_timeout = find_edge_ac_.waitForResult(ros::Duration(120.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = find_edge_ac_.getState();
      ROS_INFO("FindEdge action finished: %s",state.toString().c_str());
    }
    else
    {
      ROS_INFO("FindEdge action did not finish before the time out.");
      ros::shutdown();
    }
    
    edge_point_ = find_edge_ac_.getResult()->edge_point;
    
    edge_point_node_["vector3"].push_back(edge_point_.x);
    edge_point_node_["vector3"].push_back(edge_point_.y);
    edge_point_node_["vector3"].push_back(edge_point_.z);
  }
  
  std::string prepareMotionPhaseSpec()
  {
    // Get current phase spec and fill in necessary information
    auto goal_node = task_.getCurrentPhaseSpecNode();
    auto scope = goal_node["scope"];
    
    // Grasps
    YAML::Node tool_grasp_scope_node;
    tool_grasp_scope_node["tool-grasp"] = experiment_.tool_grasp_node;
    
    YAML::Node target_object_grasp_scope_node;
    target_object_grasp_scope_node["target-object-grasp"] = experiment_.target_object_grasp_node;
    
    // Edge point
    YAML::Node edge_point_scope_node;
    edge_point_scope_node["edge-point"] = edge_point_node_;
    
    // Put new data in the front of the scope
    // Is there a better way of doing that?
    YAML::Node new_scope;
    new_scope.push_back(tool_grasp_scope_node);
    new_scope.push_back(target_object_grasp_scope_node);
    new_scope.push_back(edge_point_scope_node);
    
    for (const auto n : scope)
    {
      new_scope.push_back(n);
    }
    
    // Replace scope
    goal_node["scope"] = new_scope;
    
    // Convert spec to string
    YAML::Emitter out;
    out << goal_node;
    std::string spec{out.c_str()};
    
    return spec;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_executive");
  TaskExecutive executive;
  executive.start();
  ros::spin();

  return 0;
}
