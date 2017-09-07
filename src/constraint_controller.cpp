#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skill_transfer/MoveArmAction.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
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
    sub_ = nh_.subscribe("/gazebo/link_states", 1, &ConstraintController::analysisCB, this);
    pub_ = nh_.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1);
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
    constraints_ = as_.acceptNewGoal()->constraints;

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

  void analysisCB(const gazebo_msgs::LinkStatesConstPtr &msg)
  {

    // Link state map
    auto link_state = toMap<std::string, geometry_msgs::Pose>(msg->name, msg->pose);

    auto knife_world_pose = link_state.find("knife::link")->second;
    auto gripper_world_pose = link_state.find("gripper::link")->second;
    auto frying_pan_world_pose = link_state.find("frying_pan::link")->second;
    
    gazebo_msgs::LinkState cmd;
    cmd.link_name = "gripper::link";
    cmd.reference_frame = "world";
    cmd.pose = gripper_world_pose;

    // When action is not active send zero twist,
    // otherwise do all the calculations
    if (as_.isActive())
    {
      // Prepare controller inputs

      KDL::Frame knife_world_frame;
      tf::poseMsgToKDL(knife_world_pose, knife_world_frame);

      KDL::Frame gripper_world_frame;
      tf::poseMsgToKDL(gripper_world_pose, gripper_world_frame);

      // Knife in gripper space
      auto knife_gripper_frame = gripper_world_frame.Inverse() * knife_world_frame;

      Eigen::VectorXd inputs(18);
      inputs.segment(0, 6) = msgPoseToEigenVector(gripper_world_pose);
      inputs.segment(6, 6) = msgPoseToEigenVector(frying_pan_world_pose);
      inputs.segment(12, 6) = kdlFrameToEigenVector(knife_gripper_frame);

      // Start the controller if it's a new one
      if (!controller_started_)
      {
        // FIXME: get nWSR from parameter server
        if (!controller_.start(inputs, 100))
        {
          throw std::runtime_error("Failed to start controller.");
        }

        controller_started_ = true;
      }

      // Get new calculations from the controller
      // FIXME: get nWSR from parameter server
      if (!controller_.update(inputs, 100))
      {
        throw std::runtime_error("Failed to update controller.");
      }

      // Insert the Jacobian to the message as twist
      auto jacobian = getJacobian(controller_, "gripper-frame", inputs).data * controller_.get_command();
      cmd.twist = eigenVectorToMsgTwist(jacobian);

      // Visualization
      pub_viz_.publish(createPointMarker(controller_, "knife-base", "world"));
      pub_viz_.publish(createPointMarker(controller_, "frying-pan-edge", "world"));
      pub_viz_.publish(createPointDirectionMarker(controller_, "knife-base", "knife-pan-distance", "world"));
    }
    
    pub_.publish(cmd);

    // ROS_INFO_STREAM("Twist: " << cmd.twist);
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<skill_transfer::MoveArmAction> as_;
  std::string action_name_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_viz_;
  std::string constraints_;
  giskard_core::QPController controller_;
  bool controller_started_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "constraint_controller");

  ConstraintController controller("move_arm");
  ros::spin();

  return 0;
}
