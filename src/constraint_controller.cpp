#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skill_transfer/MoveArmAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <giskard_core/giskard_core.hpp>
#include "skill_transfer/conversions.h"
#include "skill_transfer/giskard_utils.h"
#include "skill_transfer/giskard_viz.h"

class ConstraintController
{
public:
  ConstraintController(std::string name) : as_(nh_, name, false),
                                           action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ConstraintController::onGoal, this));
    as_.registerPreemptCallback(boost::bind(&ConstraintController::onPreempt, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/joint_states", 1, &ConstraintController::onJointStatesMsg, this);
    
    // Topic for real PR2 commands (joint velocities)
    pub_ = nh_.advertise<sensor_msgs::JointState>("/pr2/commands", 1);
    // Topic for simulation and executive node, since they only
    // care about the end effector velocity and not about joint velocities
    pub_gripper_ = nh_.advertise<geometry_msgs::Twist>("/set_ee_twist", 1);
    // Desired motion state visualization for RViz
    pub_viz_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    as_.start();
  }

  ~ConstraintController()
  {
  }

  void onGoal()
  {
    // Accept goal and get new constraints
    const auto goal = as_.acceptNewGoal();
    constraints_ = goal->constraints;
    gripper_link_name_ = goal->gripper_link_name;
    tool_link_name_ = goal->tool_link_name;
    utility_link_name_ = goal->utility_link_name;  

    ROS_INFO("%s: Received a new goal", action_name_.c_str());
    // ROS_INFO_STREAM(constraints_);
    // Clear the previous state
    controller_started_ = false;
    controller_ = generateController(constraints_);
  }

  void onPreempt()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void onJointStatesMsg(const sensor_msgs::JointStateConstPtr &msg)
  {

    // Link state map
    auto joint_positions = toMap<std::string, double>(msg->name, msg->position);
    auto joint_velocities = toMap<std::string, double>(msg->name, msg->velocity);    

    auto torso_lift_joint_position = joint_positions.find("torso_lift_joint")->second;
    auto l_shoulder_pan_joint_position = joint_positions.find("l_shoulder_pan_joint")->second;
    auto l_shoulder_lift_joint_position = joint_positions.find("l_shoulder_lift_joint")->second;
    auto l_upper_arm_roll_joint_position = joint_positions.find("l_upper_arm_roll_joint")->second;
    auto l_elbow_flex_joint_position = joint_positions.find("l_elbow_flex_joint")->second;
    auto l_forearm_roll_joint_position = joint_positions.find("l_forearm_roll_joint")->second;
    auto l_wrist_flex_joint_position = joint_positions.find("l_wrist_flex_joint")->second;
    auto l_wrist_roll_joint_position = joint_positions.find("l_wrist_roll_joint")->second;
    auto r_shoulder_pan_joint_position = joint_positions.find("r_shoulder_pan_joint")->second;
    auto r_shoulder_lift_joint_position = joint_positions.find("r_shoulder_lift_joint")->second;
    auto r_upper_arm_roll_joint_position = joint_positions.find("r_upper_arm_roll_joint")->second;
    auto r_elbow_flex_joint_position = joint_positions.find("r_elbow_flex_joint")->second;
    auto r_forearm_roll_joint_position = joint_positions.find("r_forearm_roll_joint")->second;
    auto r_wrist_flex_joint_position = joint_positions.find("r_wrist_flex_joint")->second;
    auto r_wrist_roll_joint_position = joint_positions.find("r_wrist_roll_joint")->second;

    auto torso_lift_joint_velocity = joint_velocities.find("torso_lift_joint")->second;
    auto l_shoulder_pan_joint_velocity = joint_velocities.find("l_shoulder_pan_joint")->second;
    auto l_shoulder_lift_joint_velocity = joint_velocities.find("l_shoulder_lift_joint")->second;
    auto l_upper_arm_roll_joint_velocity = joint_velocities.find("l_upper_arm_roll_joint")->second;
    auto l_elbow_flex_joint_velocity = joint_velocities.find("l_elbow_flex_joint")->second;
    auto l_forearm_roll_joint_velocity = joint_velocities.find("l_forearm_roll_joint")->second;
    auto l_wrist_flex_joint_velocity = joint_velocities.find("l_wrist_flex_joint")->second;
    auto l_wrist_roll_joint_velocity = joint_velocities.find("l_wrist_roll_joint")->second;
    auto r_shoulder_pan_joint_velocity = joint_velocities.find("r_shoulder_pan_joint")->second;
    auto r_shoulder_lift_joint_velocity = joint_velocities.find("r_shoulder_lift_joint")->second;
    auto r_upper_arm_roll_joint_velocity = joint_velocities.find("r_upper_arm_roll_joint")->second;
    auto r_elbow_flex_joint_velocity = joint_velocities.find("r_elbow_flex_joint")->second;
    auto r_forearm_roll_joint_velocity = joint_velocities.find("r_forearm_roll_joint")->second;
    auto r_wrist_flex_joint_velocity = joint_velocities.find("r_wrist_flex_joint")->second;
    auto r_wrist_roll_joint_velocity = joint_velocities.find("r_wrist_roll_joint")->second;

    // When action is not active send zero twist,
    // otherwise do all the calculations
    if (as_.isActive())
    {
      // Prepare controller inputs

      Eigen::VectorXd inputs(15);
      inputs(0) = torso_lift_joint_position;
      inputs(1) = l_shoulder_pan_joint_position;
      inputs(2) = l_shoulder_lift_joint_position;
      inputs(3) = l_upper_arm_roll_joint_position;
      inputs(4) = l_elbow_flex_joint_position;
      inputs(5) = l_forearm_roll_joint_position;
      inputs(6) = l_wrist_flex_joint_position;
      inputs(7) = l_wrist_roll_joint_position;
      inputs(8) = r_shoulder_pan_joint_position;
      inputs(9) = r_shoulder_lift_joint_position;
      inputs(10) = r_upper_arm_roll_joint_position;
      inputs(11) = r_elbow_flex_joint_position;
      inputs(12) = r_forearm_roll_joint_position;
      inputs(13) = r_wrist_flex_joint_position;
      inputs(14) = r_wrist_roll_joint_position;

      Eigen::VectorXd velocities(15);
      velocities(0) = torso_lift_joint_velocity;
      velocities(1) = l_shoulder_pan_joint_velocity;
      velocities(2) = l_shoulder_lift_joint_velocity;
      velocities(3) = l_upper_arm_roll_joint_velocity;
      velocities(4) = l_elbow_flex_joint_velocity;
      velocities(5) = l_forearm_roll_joint_velocity;
      velocities(6) = l_wrist_flex_joint_velocity;
      velocities(7) = l_wrist_roll_joint_velocity;
      velocities(8) = r_shoulder_pan_joint_velocity;
      velocities(9) = r_shoulder_lift_joint_velocity;
      velocities(10) = r_upper_arm_roll_joint_velocity;
      velocities(11) = r_elbow_flex_joint_velocity;
      velocities(12) = r_forearm_roll_joint_velocity;
      velocities(13) = r_wrist_flex_joint_velocity;
      velocities(14) = r_wrist_roll_joint_velocity;

      // Start the controller if it's a new one
      if (!controller_started_)
      {
        // FIXME: get nWSR from parameter server
        if (!controller_.start(inputs, 100))
        {
          throw std::runtime_error("Failed to start controller.");
        }
        
        ROS_INFO("starting controller");
        controller_started_ = true;
      }

      // Get new calculations from the controller
      if (!controller_.update(inputs, 100))
      {
        throw std::runtime_error("Failed to update controller.");
      }

      
      // Insert the Jacobian to the message as twist
      const Eigen::VectorXd desired_velocity = 
              getJacobian(controller_, "gripper-frame", inputs).data * controller_.get_command();
      auto ee_twist_desired = eigenVectorToMsgTwist(desired_velocity);
      auto cmd = eigenVectorToMsgJointState(controller_.get_command());

      pub_.publish(cmd);
      pub_gripper_.publish(ee_twist_desired);

      // TODO: Rather than distance this should generically post all defined positions      
      // Calculate distance for feedback
      const KDL::Expression<KDL::Vector>::Ptr point_exp =
          controller_.get_scope().find_vector_expression("tool-point");
      const KDL::Expression<KDL::Vector>::Ptr direction_exp =
          controller_.get_scope().find_vector_expression("utility-point");
      const KDL::Expression<KDL::Vector>::Ptr distance_exp =
          controller_.get_scope().find_vector_expression("distance");
          
      // KDL::Vector tool_point = point_exp->value();
      // KDL::Vector utility_point = direction_exp->value();
      auto distance_vector = distance_exp->value();

      //double distance = (utility_point - tool_point).Norm();
      double distance = distance_vector.Norm();
      
      //ROS_INFO_STREAM("Distance: " << distance << " " << distance2);

      feedback_.distance = distance;
      
      as_.publishFeedback(feedback_);
          

      // Visualization
      pub_viz_.publish(createPointMarker(controller_, "tool-point", "base_footprint"));
      pub_viz_.publish(createPointMarker(controller_, "utility-point", "base_footprint"));
      pub_viz_.publish(createPointDirectionMarker(controller_, "tool-point", "distance", "base_footprint"));
    }
    else
    {
      Eigen::VectorXd inputs(15);
      inputs(0) = 0.0;
      inputs(1) = 0.0;
      inputs(2) = 0.0;
      inputs(3) = 0.0;
      inputs(4) = 0.0;
      inputs(5) = 0.0;
      inputs(6) = 0.0;
      inputs(7) = 0.0;
      inputs(8) = 0.0;
      inputs(9) = 0.0;
      inputs(10) = 0.0;
      inputs(11) = 0.0;
      inputs(12) = 0.0;
      inputs(13) = 0.0;
      inputs(14) = 0.0;
      
      auto cmd = eigenVectorToMsgJointState(inputs);

      pub_.publish(cmd);
    } 

    // ROS_INFO_STREAM("Twist: " << cmd.twist);
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<skill_transfer::MoveArmAction> as_;
  std::string action_name_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_gripper_;
  ros::Publisher pub_gripper_measured_;
  ros::Publisher pub_viz_;
  std::string constraints_;
  giskard_core::QPController controller_;
  bool controller_started_;
  skill_transfer::MoveArmFeedback feedback_;
  std::string gripper_link_name_;
  std::string tool_link_name_;
  std::string utility_link_name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "constraint_controller");

  ConstraintController controller("move_arm");
  ros::spin();

  return 0;
}
