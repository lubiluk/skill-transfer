#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skill_transfer/FindEdgeAction.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <cstdlib>
#include <boost/format.hpp>
#include <fstream>
#include <vector>
#include <algorithm>
#include <sstream>


class EdgeDetector 
{  
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<skill_transfer::FindEdgeAction> as_;
  std::string action_name_;
  skill_transfer::FindEdgeResult result_;
  std::string point_cloud_directory_path_;
  
public:
  EdgeDetector(std::string name): as_(nh_, name, false), 
                                  action_name_(name)
  {
    if ( !nh_.getParam("point_cloud_directory", point_cloud_directory_path_) )
    {
      throw std::runtime_error("Could not find parameter 'point_cloud_directory' in namespace '" + nh_.getNamespace() + "'.");
    }
  
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&EdgeDetector::onGoal, this));
    as_.registerPreemptCallback(boost::bind(&EdgeDetector::onPreempt, this));
    
    as_.start();
  }
  
  void onGoal()
  {
    // Accept goal and get the reference point
    const auto goal = as_.acceptNewGoal();
    const geometry_msgs::Point &reference_point = goal->reference_point;
    const std::string &point_cloud_file_name = goal->point_cloud_file_name;
    const std::string point_cloud_path = point_cloud_directory_path_ + point_cloud_file_name;
    
    ROS_INFO_STREAM("Reference point: " << reference_point.x << " " << reference_point.y << " " << reference_point.z);
    
    const auto command = boost::format("run_edge_detector.sh /usr/local/MATLAB/MATLAB_Runtime/v92 %1% \"[%2% %3% %4%]\" > edge.txt") 
                         % point_cloud_path % reference_point.x % reference_point.y % reference_point.z;
    
    ROS_INFO_STREAM("Command: " << command);
    
    std::system(command.str().c_str());
    
    std::ifstream file("edge.txt");
    
    std::string output;
    
    
    for (std::string line; std::getline(file, line);)
    {
      ROS_INFO_STREAM(line);
    
      if (line.empty())
        continue;
        
      output = line;
    }
    
    std::istringstream iss(output);
    
    double x, y, z;
    
    geometry_msgs::Point point;

    iss >> point.x;
    iss >> point.y;
    iss >> point.z;
    
    result_.edge_point = point;
    
    ROS_INFO_STREAM("Edge point: " << point.x << " " << point.y << " " << point.z);
    
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }

  void onPreempt()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "edge_detector");

  EdgeDetector detector("find_edge");
  ros::spin();

  return 0;
}
