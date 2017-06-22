#include <ros/ros.h>
#include <algorithm>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>

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
      set_link_state_sub_(nh_.subscribe("/gazebo/set_link_state", 1, &TaskExecutive::setLinkStateCallback, this)) 
      {}

    ~TaskExecutive() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber link_state_sub_;
    ros::Subscriber set_link_state_sub_;

    void linkStateCallback(const gazebo_msgs::LinkStatesConstPtr& msg)
    {
      auto link_twists = to_map<std::string, geometry_msgs::Twist>(msg->name, msg->twist);
      auto gripper_pose = link_twists["gripper::link"];

      ROS_INFO_STREAM("Gripper twist: " << gripper_pose << "\n");
    }

    void setLinkStateCallback(const gazebo_msgs::LinkStateConstPtr& msg)
    {
      ROS_INFO_STREAM("Set gripper twist: " << msg->twist << "\n");
    }
};

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "task_executive");
  ros::NodeHandle nh("~");

  try
  {
    TaskExecutive te(nh);
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
