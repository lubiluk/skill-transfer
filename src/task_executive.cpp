#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ContactsState.h>
#include <giskard_msgs/WholeBodyAction.h>

#include <skill_transfer/StopCondition.h>
#include <skill_transfer/GetTaskSpec.h>
#include <skill_transfer/GetMotionSpec.h>

#include "skill_transfer/twist_log.h"

class TaskExecutive
{
private:
  // Possible internal states of the node
  enum State
  {
    Created,
    Initialized,
    Waiting,
    ObtainingTaskSpec,
    Ready,
    ObtainingMotionSpec,
    Running,
    Stopped,
    Finished
  };
  // State
  State state_ = State::Created;
  // ROS handles
  ros::NodeHandle node_handle_;
  ros::Subscriber ee_twist_subscriber_;
  ros::Subscriber set_ee_twist_subscriber_;
  ros::Subscriber tool_contact_subscriber_;
  ros::ServiceClient task_spec_service_client_;
  ros::ServiceClient motion_spec_service_client_;
  actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction> body_action_client_;
  // Motion control variables
  int phase_count_;
  int phase_index_;
  TwistLog velocity_log_;
  TwistLog command_log_;
  double goal_distance_;
  skill_transfer::StopCondition stop_condition_;
  std::string spec_;

public:
  TaskExecutive() : node_handle_("~"),
                    body_action_client_("qp_controller/command", true),
                    velocity_log_(10),
                    command_log_(10)
  {
    ee_twist_subscriber_ = node_handle_.subscribe("/l_ee_twist", 1,
                                                  &TaskExecutive::onEeTwistMsg, this);
    set_ee_twist_subscriber_ = node_handle_.subscribe("/set_l_ee_twist", 1,
                                                      &TaskExecutive::onSetEeTwistMsg, this);

    tool_contact_subscriber_ = node_handle_.subscribe("/tool_contact_sensor_state", 1,
                                                      &TaskExecutive::onToolContactSensorStateMsg, this);

    task_spec_service_client_ = node_handle_.serviceClient<skill_transfer::GetTaskSpec>("/knowledge_manager/get_task_spec");
    motion_spec_service_client_ = node_handle_.serviceClient<skill_transfer::GetMotionSpec>("/knowledge_manager/get_motion_spec");

    state_ = State::Initialized;
  }

  void start()
  {
    ROS_ASSERT(state_ == State::Initialized);
    
    // Wait for the 3rd parties
    state_ = State::Waiting;

    task_spec_service_client_.waitForExistence();
    motion_spec_service_client_.waitForExistence();
    body_action_client_.waitForServer();

    // Obtain the number of phases
    state_ = State::ObtainingTaskSpec;

    skill_transfer::GetTaskSpec srv;

    if (!task_spec_service_client_.call(srv))
    {
      throw std::runtime_error("Failed to call service get_task_spec");
    }

    phase_count_ = srv.response.motion_phase_count;

    state_ = State::Ready;

    ROS_INFO("Press any key to begin the motion");
  
    std::getchar(); 

    // Start the motion
    startPhase(0);
  }

  void onEeTwistMsg(const geometry_msgs::TwistConstPtr &msg)
  {
    if (state_ != State::Running)
    {
      return;
    }

    // Save twist to log
    velocity_log_.push(*msg);

    checkMeasuredVelocityStop();
  }

  void onSetEeTwistMsg(const geometry_msgs::TwistConstPtr &msg)
  {
    // Do not track velocities until the motion starts
    if (state_ != State::Running)
    {
      return;
    }

    // Save twist to log
    command_log_.push(*msg);

    checkDesiredVelocityStop();
  }

  void onToolContactSensorStateMsg(const gazebo_msgs::ContactsStatePtr
                                       &msg)
  {
    // Do not track contact until the motion starts
    if (state_ != State::Running)
    {
      return;
    }

    // Continue only when there's a contact
    if (msg->states.size() == 0)
      return;

    checkContactStop();
  }

  void onFinish(const actionlib::SimpleClientGoalState &state,
                const giskard_msgs::WholeBodyResultConstPtr &result)
  {
    // This should never happen, as constraint_controller doesn't
    // ever finish.
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    completePhase();
  }

  void onFeedback(const giskard_msgs::WholeBodyFeedbackConstPtr &feedback)
  {
    goal_distance_ = 0.0;
  }

private:
  void startPhase(int index)
  {
    ROS_ASSERT(index >= 0 && index < phase_count_);
    ROS_ASSERT(state_ == State::Ready);

    state_ = State::ObtainingMotionSpec;

    // Obtain the motion spec
    skill_transfer::GetMotionSpec srv;

    srv.request.index = index;

    if (!motion_spec_service_client_.call(srv))
    {
      throw std::runtime_error("Failed to call service get_task_spec");
    }

    spec_ = srv.response.spec;
    stop_condition_ = srv.response.stop_condition;

    state_ = State::Stopped;

    phase_index_ = index;

    goal_distance_ = std::numeric_limits<double>::infinity();
    velocity_log_.clear();
    command_log_.clear();

    // Create and send goal
    giskard_msgs::WholeBodyGoal goal;
    goal.command.type = 1;
    goal.command.yaml_spec = spec_;
    // ROS_INFO("Spec:");
    // ROS_INFO_STREAM(spec_);

    ROS_INFO("Sending new goal.");

    body_action_client_
        .sendGoal(goal,
                  boost::bind(&TaskExecutive::onFinish, this, _1, _2),
                  actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction>::SimpleActiveCallback(),
                  boost::bind(&TaskExecutive::onFeedback, this, _1));

    state_ = State::Running;
  }

  void finish()
  {
    body_action_client_.cancelGoal();

    state_ = State::Finished;
  }

  void checkDesiredVelocityStop()
  {
    if (goal_distance_ > stop_condition_.activation_distance)
    {
      return;
    }

    if (!command_log_.allFilledAndBelowThreshold(stop_condition_.desired_velocity_min))
    {
      return;
    }

    ROS_INFO("Desired Velocity Stop");

    completePhase();
  }

  void checkMeasuredVelocityStop()
  {
    if (goal_distance_ > stop_condition_.activation_distance)
    {
      return;
    }

    if (!velocity_log_.allFilledAndBelowThreshold(stop_condition_.measured_velocity_min))
    {
      return;
    }

    ROS_INFO("Measured Velocity Stop");

    completePhase();
  }

  void checkContactStop()
  {
    if (!stop_condition_.contact)
    {
      return;
    }

    if (goal_distance_ > stop_condition_.activation_distance)
    {
      return;
    }

    ROS_INFO_STREAM("Contact Stop");

    completePhase();
  }

  void completePhase()
  {
    state_ = State::Stopped;

    int next_phase_index = phase_index_ + 1;

    state_ = State::Ready;

    if (phase_count_ > next_phase_index)
    {
      ROS_INFO("Next");
      startPhase(next_phase_index);
    }
    else
    {
      ROS_INFO("Finish");
      finish();
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
