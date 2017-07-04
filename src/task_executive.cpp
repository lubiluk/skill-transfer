#include <ros/ros.h>
#include <algorithm>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <skill_transfer/MoveArmAction.h>

template<class T, class U>
inline std::map<T, U> to_map(const std::vector<T>& keys, const std::vector<U>& values)
{
  if (keys.size() != values.size())
    throw std::runtime_error("Number of keys not equal to numbers of values.");

  std::map<T, U> result;
  for (size_t i=0; i<keys.size(); ++i)
    result.insert(std::pair<T,U>(keys[i], values[i]));

  return result;
}


class TaskExecutive
{
  public:
    TaskExecutive(const ros::NodeHandle& nh): 
      nh_(nh),
      link_state_sub_(nh_.subscribe("/gazebo/link_states", 1, &TaskExecutive::linkStateCallback, this)),
      set_link_state_sub_(nh_.subscribe("/gazebo/set_link_state", 1, &TaskExecutive::setLinkStateCallback, this)),
      ac_("move_arm", true)
      {}

    ~TaskExecutive() {}
    
    void start() {
      this->requestMotion();
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber link_state_sub_;
    ros::Subscriber set_link_state_sub_;
    double stopVelocity = 1.0e-05;
    actionlib::SimpleActionClient<skill_transfer::MoveArmAction> ac_;

    void linkStateCallback(const gazebo_msgs::LinkStatesConstPtr& msg)
    {
      auto link_twists = to_map<std::string, geometry_msgs::Twist>(msg->name, msg->twist);
      auto gripper_twist = link_twists["gripper::link"];
      
      if (gripper_twist.linear.x < stopVelocity &&
          gripper_twist.linear.y < stopVelocity &&
          gripper_twist.linear.z < stopVelocity) {
        ROS_INFO_STREAM("Gripper stop! " << gripper_twist.linear << "\n"); 
      }
    }

    void setLinkStateCallback(const gazebo_msgs::LinkStateConstPtr& msg)
    {
      if (msg->link_name != "gripper::link") return;
      
      if (msg->twist.linear.x < stopVelocity &&
          msg->twist.linear.y < stopVelocity &&
          msg->twist.linear.z < stopVelocity) {
        ROS_INFO_STREAM("Command stop! :" << msg->twist.linear << "\n"); 
      }
      
    }

    void requestMotion()
    {
      ROS_INFO("Waiting for server to start.");

      ac_.waitForServer();

      ROS_INFO("Action server started, sending goal.");

      skill_transfer::MoveArmGoal goal;
      goal.arm_id = 1;
      ac_.sendGoal(goal);

      // wait for the action to return
      bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout) 
      {
        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
      }
      else
        ROS_INFO("Action did not finish before the timeout.");

    }
};

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "task_executive");
  ros::NodeHandle nh("~");

  try
  {
    TaskExecutive te(nh);
    te.start();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
