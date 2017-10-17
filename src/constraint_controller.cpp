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
    as_.registerGoalCallback(boost::bind(&ConstraintController::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ConstraintController::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/joint_states", 1, &ConstraintController::analysisCB, this);
    
    pub_ = nh_.advertise<sensor_msgs::JointState>("/pr2/commands", 1);
    pub_gripper_ = nh_.advertise<geometry_msgs::Twist>("/gripper_twist", 1);
    // TODO: Make an independent node from this
    pub_viz_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    as_.start();
  }

  ~ConstraintController()
  {
  }

  void goalCB()
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

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const sensor_msgs::JointStateConstPtr &msg)
  {

    // Link state map
    auto link_state = toMap<std::string, double>(msg->name, msg->position);

    auto torso_lift_joint_position = link_state.find("torso_lift_joint")->second;
    auto l_shoulder_pan_joint_position = link_state.find("l_shoulder_pan_joint")->second;
    auto l_shoulder_lift_joint_position = link_state.find("l_shoulder_lift_joint")->second;
    auto l_upper_arm_roll_joint_position = link_state.find("l_upper_arm_roll_joint")->second;
    auto l_elbow_flex_joint_position = link_state.find("l_elbow_flex_joint")->second;
    auto l_forearm_roll_joint_position = link_state.find("l_forearm_roll_joint")->second;
    auto l_wrist_flex_joint_position = link_state.find("l_wrist_flex_joint")->second;
    auto l_wrist_roll_joint_position = link_state.find("l_wrist_roll_joint")->second;
    auto r_shoulder_pan_joint_position = link_state.find("r_shoulder_pan_joint")->second;
    auto r_shoulder_lift_joint_position = link_state.find("r_shoulder_lift_joint")->second;
    auto r_upper_arm_roll_joint_position = link_state.find("r_upper_arm_roll_joint")->second;
    auto r_elbow_flex_joint_position = link_state.find("r_elbow_flex_joint")->second;
    auto r_forearm_roll_joint_position = link_state.find("r_forearm_roll_joint")->second;
    auto r_wrist_flex_joint_position = link_state.find("r_wrist_flex_joint")->second;
    auto r_wrist_roll_joint_position = link_state.find("r_wrist_roll_joint")->second;


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
      // FIXME: get nWSR from parameter server
      if (!controller_.update(inputs, 100))
      {
        throw std::runtime_error("Failed to update controller.");
      }

      
      // Insert the Jacobian to the message as twist
      const Eigen::VectorXd jacobian = getJacobian(controller_, "gripper-frame", inputs).data * controller_.get_command();
      auto gripper_twist = eigenVectorToMsgTwist(jacobian);
      auto cmd = eigenVectorToMsgJointState(controller_.get_command());

      pub_.publish(cmd);
      pub_gripper_.publish(gripper_twist);

      // TODO: Rather than distance this should generically post all defined positions      
      // Calculate distance for feedback
      const KDL::Expression<KDL::Vector>::Ptr point_exp =
          controller_.get_scope().find_vector_expression("tool-point");
      const KDL::Expression<KDL::Vector>::Ptr direction_exp =
          controller_.get_scope().find_vector_expression("utility-point");
          
          
      auto point = point_exp->value();
      auto direction = direction_exp->value();
      auto distance = (direction - point).Norm();
      
      feedback_.distance = distance;
      
      as_.publishFeedback(feedback_);
          

      // Visualization
      pub_viz_.publish(createPointMarker(controller_, "tool-point", "world"));
      pub_viz_.publish(createPointMarker(controller_, "utility-point", "world"));
      pub_viz_.publish(createPointDirectionMarker(controller_, "tool-point", "distance", "world"));
    }
    else
    {
      sensor_msgs::JointState cmd;

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
