#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skill_transfer/MoveArmAction.h>

class ConstraintController 
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<skill_transfer::MoveArmAction> as_;
    std::string action_name_;
    skill_transfer::MoveArmFeedback feedback_;
    skill_transfer::MoveArmResult result_;

  public:
    ConstraintController(std::string name):
      as_(nh_, name, boost::bind(&ConstraintController::executeAction, this, _1), false),
      action_name_(name)
    {
      as_.start();
    }

    ~ConstraintController(void)
    {
    }

    void executeAction(const skill_transfer::MoveArmGoalConstPtr &goal)
    {
      ROS_INFO("Executing action");
      feedback_.goal_distance = 0.0;
      result_.is_goal_reached = true;
      as_.setSucceeded(result_);
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constraint_controller");

  ROS_INFO("Waiting for requests");

  ConstraintController controller("move_arm");
  ros::spin();

  return 0;
}
