#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <skill_transfer/MoveArmAction.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include "skill_transfer/conversions.h"
#include "skill_transfer/file_sequence.h"
#include "skill_transfer/twist_log.h"
#include <gazebo_msgs/ContactsState.h>

class TaskExecutive
{
public:
  TaskExecutive() : ac_("move_arm", true),
                    file_sequence_("config"),
                    velocity_log_(10),
                    command_log_(10)
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

    ROS_INFO("Motion files:");

    for (auto filename : file_sequence_.getFilenames())
    {
      ROS_INFO_STREAM(filename);
    }
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
    auto gripper_twist = link_twists["gripper::link"];

    // Save twist to log
    velocity_log_.push(gripper_twist);

//    ROS_INFO_STREAM("Velocity: " << gripper_twist);

    // Check progress for stop condition
    checkProgress();
  }

  void setLinkStateAnalysisCB(const gazebo_msgs::LinkStateConstPtr &msg)
  {
    // Do not track velocities until the motion starts
    if (!running_)
      return;

    if (msg->link_name != "gripper::link")
      return;

    // Save twist to log
    command_log_.push(msg->twist);

//    ROS_INFO_STREAM("Command: " << msg->twist);

    // Check progress for stop condition
    // TODO: Is it necessary to call it here as well?
    checkProgress();
  }
  
  void toolContactSensorStateAnalysisCB(const gazebo_msgs::ContactsStatePtr
   &msg)
  {
    // Do not track contact until the motion starts
    if (!running_)
      return;
      
    if (goal_distance_ > 0.02) 
      return;
      
    //TODO: this is cheating
    if (msg->states.size() > 0 && file_sequence_.hasNextFile()) {
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
  FileSequence file_sequence_;
  TwistLog velocity_log_;
  TwistLog command_log_;
  double goal_distance_;

  void sendNextGoal()
  {
    velocity_log_.clear();
    command_log_.clear();

    skill_transfer::MoveArmGoal goal;
    goal.constraints = file_sequence_.getNextFileContents();

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
    if (goal_distance_ > 0.02) 
      return;
    
    if (!velocity_log_.allFilledAndBelowThreshold(0.0001) &&
        !command_log_.allFilledAndBelowThreshold(0.0001))
      return;
      
    ROS_INFO("Velocity/Command stop");

    completeStage();
  }
  
  void completeStage()
  {
    running_ = false;
    goal_distance_ = 1000.0;
  
    if (file_sequence_.hasNextFile())
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
