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

class TaskExecutive
{
public:
  TaskExecutive() : nh_("~"),
                    ac_("move_arm", true),
                    velocity_log_(10),
                    command_log_(10)
  {
    // Get task specifications
    if ( !nh_.getParam("task_directory", task_directory_path_) )
    {
      throw std::runtime_error("Could not find parameter 'task_directory' in namespace '" + nh_.getNamespace() + "'.");
    }
    
    std::string motion_directory;
    // Get task specifications
    if ( !nh_.getParam("motion_directory", motion_directory) )
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
    task_.motion_directory_path = motion_directory;
    task_.load(task_file_path, motiom_template_file_path);
    
    ROS_INFO_STREAM("Task: " << task_.name);
    ROS_INFO_STREAM("Phases: " << task_.phases.size());
  
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

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

  void sendNextGoal()
  {
    velocity_log_.clear();
    command_log_.clear();

    skill_transfer::MoveArmGoal goal;
    goal.constraints = task_.getCurrentPhaseSpec();

    ROS_INFO("Sending new goal.");

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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_executive");
  TaskExecutive executive;
  executive.start();
  ros::spin();

  return 0;
}
