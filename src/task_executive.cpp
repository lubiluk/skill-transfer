#include "skill_transfer/conversions.h"
#include "skill_transfer/twist_log.h"
#include "skill_transfer/task.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <skill_transfer/MoveArmAction.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ContactsState.h>

class TaskExecutive
{
public:
  TaskExecutive() : ac_("move_arm", true),
                    velocity_log_(10),
                    command_log_(10),
                    task_("tasks/scraping_butter.yaml")
  {
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

    link_state_sub_ = nh_.subscribe("/gazebo/link_states", 1,
                                    &TaskExecutive::linkStateAnalysisCB, this);
    set_link_state_sub_ = nh_.subscribe("/gazebo/set_link_state", 1,
                                        &TaskExecutive::setLinkStateAnalysisCB, this);
                                        
    tool_contact_sensor_state_sub_ = nh_.subscribe("/tool_contact_sensor_state", 1,
                                                   &TaskExecutive::toolContactSensorStateAnalysisCB, this);
    
    ROS_INFO_STREAM("Task: " << task_.name);
    ROS_INFO_STREAM("Phases: " << task_.phases.size());
  }

  ~TaskExecutive()
  {
  }

  void start()
  {
    sendNextGoal();
  }

  void finishCB(const actionlib::SimpleClientGoalState &state,
                const skill_transfer::MoveArmResultConstPtr &result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ros::shutdown();
  }

  void feedbackCB(const skill_transfer::MoveArmFeedbackConstPtr &feedback)
  {
    goal_distance_ = feedback->distance;
  }

  void linkStateAnalysisCB(const gazebo_msgs::LinkStatesConstPtr &msg)
  {
    if (!running_)
      return;

    auto link_twists = toMap<std::string, geometry_msgs::Twist>(msg->name, msg->twist);
    auto gripper_twist = link_twists[task_.scene_objects.gripper_link_name];

    // Save twist to log
    velocity_log_.push(gripper_twist);

    checkProgress();
  }

  void setLinkStateAnalysisCB(const gazebo_msgs::LinkStateConstPtr &msg)
  {
    // Do not track velocities until the motion starts
    if (!running_)
      return;

    if (msg->link_name != task_.scene_objects.gripper_link_name)
      return;

    // Save twist to log
    command_log_.push(msg->twist);

    checkProgress();
  }
  
  void toolContactSensorStateAnalysisCB(const gazebo_msgs::ContactsStatePtr
   &msg)
  {
    // Do not track contact until the motion starts
    if (!running_)
      return;
      
    const auto &phase = task_.getCurrentPhase();
      
    if ( !phase.stop_condition.contact )
      return;
      
    if (goal_distance_ > phase.stop_activation_distance) 
      return;
      
    if (msg->states.size() > 0) {
      ROS_INFO_STREAM("Contact stop");
      completeStage();
    }
  }

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
  Task task_;

  void sendNextGoal()
  {
    velocity_log_.clear();
    command_log_.clear();

    skill_transfer::MoveArmGoal goal;
    goal.gripper_link_name = task_.scene_objects.gripper_link_name;
    goal.tool_link_name = task_.scene_objects.tool_link_name;
    goal.utility_link_name = task_.scene_objects.utility_link_name;
    goal.constraints = task_.getCurrentPhaseSpec();

    ROS_INFO("Sending new goal.");

    ac_.sendGoal(goal,
                 boost::bind(&TaskExecutive::finishCB, this, _1, _2),
                 actionlib::SimpleActionClient<skill_transfer::MoveArmAction>::SimpleActiveCallback(),
                 boost::bind(&TaskExecutive::feedbackCB, this, _1));

    running_ = true;
  }

  void cancelCurrentGoal()
  {
    ac_.cancelGoal();
    running_ = false;
  }

  void checkProgress()
  {
    const auto &phase = task_.getCurrentPhase();
  
    if (goal_distance_ > phase.stop_activation_distance) 
      return;
    
    if (!velocity_log_.allFilledAndBelowThreshold(phase.stop_condition.measured_velocity_min) &&
        !command_log_.allFilledAndBelowThreshold(phase.stop_condition.desired_velocity_min))
      return;
      
    ROS_INFO("Velocity/Command stop");

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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_executive");
  TaskExecutive executive;
  executive.start();
  ros::spin();

  return 0;
}
