#include <ros/ros.h>
#include <algorithm>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <skill_transfer/MoveArmAction.h>
#include <fstream>
#include <deque>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

template<class T, class U>
inline std::map<T, U> to_map(const std::vector<T> &keys, const std::vector<U> &values)
{
  if (keys.size() != values.size())
    throw std::runtime_error("Number of keys not equal to numbers of values.");

  std::map<T, U> result;
  for (size_t i = 0; i < keys.size(); ++i)
    result.insert(std::pair<T, U>(keys[i], values[i]));

  return result;
}


class TaskExecutive
{
  public:
    TaskExecutive(const ros::NodeHandle &nh) :
      nh_(nh),
      link_state_sub_(nh_.subscribe("/gazebo/link_states", 1,
                                    &TaskExecutive::linkStateCallback, this)),
      set_link_state_sub_(nh_.subscribe("/gazebo/set_link_state", 1,
                                        &TaskExecutive::setLinkStateCallback, this)),
      ac_("move_arm", true),
      dir_path_("config")
    {
      ROS_INFO("Waiting for action server to start.");
      ac_.waitForServer();
      ROS_INFO("Action server started, sending goal.");

      readConstraintDir();

      for (auto &s: filenames_)
      {
        ROS_INFO_STREAM(s);
      }
    }

    ~TaskExecutive()
    {
    }

    void start()
    {
      startMotion();
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber link_state_sub_;
    ros::Subscriber set_link_state_sub_;
    double stopVelocity = 0.001;
    actionlib::SimpleActionClient<skill_transfer::MoveArmAction> ac_;
    std::deque<geometry_msgs::Twist> velocity_log_;
    std::deque<geometry_msgs::Twist> command_log_;
    unsigned int logSize_ = 10;
    bool running_ = false;
    boost::filesystem::path dir_path_;
    std::vector<std::string> filenames_;
    unsigned int current_file_index_ = 0;

    void linkStateCallback(const gazebo_msgs::LinkStatesConstPtr &msg)
    {
      if (!running_)
        return;

      auto link_twists = to_map<std::string, geometry_msgs::Twist>(msg->name, msg->twist);
      auto gripper_twist = link_twists["gripper::link"];

      // Keep the log size fixed by removing the oldest entry
      if (velocity_log_.size() >= logSize_)
        velocity_log_.pop_front();

      // Save twist to log
      velocity_log_.push_back(gripper_twist);

      ROS_INFO("Command: %lf %lf %lf",
               gripper_twist.linear.x,
               gripper_twist.linear.y,
               gripper_twist.linear.z);

      // Check progress for stop condition
      checkProgress();
    }

    void setLinkStateCallback(const gazebo_msgs::LinkStateConstPtr &msg)
    {
      if (!running_)
        return;

      if (msg->link_name != "gripper::link")
        return;

      // Keep the log size fixed by removing the oldest entry
      if (command_log_.size() >= logSize_)
        command_log_.pop_front();

      // Save twist to log
      command_log_.push_back(msg->twist);

      ROS_INFO("Velocity: %lf %lf %lf",
               msg->twist.linear.x,
               msg->twist.linear.y,
               msg->twist.linear.z);

      // Check progress for stop condition
      checkProgress();
    }

    void startMotion()
    {
      skill_transfer::MoveArmGoal goal;
      goal.constraints = readConstraintFile();

      ac_.sendGoal(goal,
                   boost::bind(&TaskExecutive::doneCallback, this, _1, _2),
                   actionlib::SimpleActionClient<skill_transfer::MoveArmAction>::SimpleActiveCallback(),
                   actionlib::SimpleActionClient<skill_transfer::MoveArmAction>::SimpleFeedbackCallback());

      running_ = true;
    }

    void stopMotion()
    {
      running_ = false;
      ac_.cancelGoal();
      velocity_log_.clear();
      command_log_.clear();
    }

    void startNextMotion()
    {
      running_ = false;
      current_file_index_ += 1;
      velocity_log_.clear();
      command_log_.clear();

      startMotion();
    }

    bool hasNextMotion()
    {
      return current_file_index_ < filenames_.size() - 1;
    }

    void checkProgress()
    {
      return;

      bool no_velocity = allBelowThreshold(velocity_log_);
      bool no_command = allBelowThreshold(command_log_);

      if (no_velocity || no_command)
      {
        // TODO: Yeah, this should probably be an iterator
        if (hasNextMotion())
        {
          // Move to the next file and start over
          ROS_INFO("Next");
          startNextMotion();
        }
        else
        {
          ROS_INFO("Stop");
//          stopMotion();
        }
      }
    }

    void doneCallback(const actionlib::SimpleClientGoalState &state,
                      const skill_transfer::MoveArmResultConstPtr &result)
    {
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      ros::shutdown();
    }

    std::string readConstraintFile()
    {
      auto path = dir_path_ /= filenames_[current_file_index_];
      boost::filesystem::ifstream file(path);
      std::stringstream buffer;
      buffer << file.rdbuf();

      return buffer.str();
    }

    void readConstraintDir()
    {
      try
      {
        for (boost::filesystem::directory_entry &f:
          boost::filesystem::directory_iterator(dir_path_))
        {
          filenames_.push_back(f.path().filename().string());
        }
      }
      catch (const boost::filesystem::filesystem_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      std::sort(filenames_.begin(), filenames_.end());
    }

    bool allBelowThreshold(std::deque<geometry_msgs::Twist> deque)
    {
      // Log has to be filled up
      if (deque.size() < logSize_)
        return false;

      return std::all_of(deque.begin(), deque.end(),
                         [this](const geometry_msgs::Twist &t) {
                           return (t.linear.x < this->stopVelocity) &&
                                  (t.linear.y < this->stopVelocity) &&
                                  (t.linear.z < this->stopVelocity);
                         });
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
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
