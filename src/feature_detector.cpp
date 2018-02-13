#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <boost/format.hpp>
#include <fstream>
#include <map>

#include <skill_transfer/DetectTargetObjectInfo.h>
#include <skill_transfer/DetectToolInfo.h>

class FeatureDetector
{
private:
  // ROS handles
  ros::NodeHandle node_handle_;
  ros::ServiceServer tool_info_service_server_;
  ros::ServiceServer target_object_info_service_server_;
  // File directories
  std::string point_cloud_directory_path_;
  std::string trained_data_directory_path_;
  // TF
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::map<std::string, std::string> name2frame_;
  // Additional parameters
  bool show_results_ = false;

public:
  FeatureDetector() : node_handle_("~"),
                      tfListener(tfBuffer)
  {
    // Initialize name -> frame map
    name2frame_["tool"] = "tool_frame";
    name2frame_["target-object"] = "target_object_frame";

    if (!node_handle_.getParam("point_cloud_directory_path", point_cloud_directory_path_))
    {
      throw std::runtime_error("Could not find parameter 'point_cloud_directory_path' in namespace '" +
                               node_handle_.getNamespace() + "'.");
    }

    if (!node_handle_.getParam("trained_data_directory_path", trained_data_directory_path_))
    {
      throw std::runtime_error("Could not find parameter 'trained_data_directory_path' in namespace '" +
                               node_handle_.getNamespace() + "'.");
    }

    node_handle_.getParam("show_results", show_results_);

    // Start services
    tool_info_service_server_ = node_handle_.advertiseService("detect_tool_info",
                                                              &FeatureDetector::serveDetectToolInfo,
                                                              this);
    target_object_info_service_server_ = node_handle_.advertiseService("detect_target_object_info",
                                                                       &FeatureDetector::serveDetectTargetObjectInfo,
                                                                       this);
  }

  bool serveDetectTargetObjectInfo(skill_transfer::DetectTargetObjectInfo::Request &req,
                                   skill_transfer::DetectTargetObjectInfo::Response &res)
  {
    // Find reference point
    const geometry_msgs::Vector3 reference_point =
        findReferencePoint("target-object", "tool");

    const std::string &point_cloud_file_name = req.point_cloud_file_name;
    const std::string point_cloud_path = point_cloud_directory_path_ + point_cloud_file_name;

    std::string display_options = "";

    display_options =  show_results_ ? "1 1" : "";

    const auto command =
        boost::format("run_get_target_obj_info.sh /usr/local/MATLAB/MATLAB_Runtime/v93 %1% \"[%2% %3% %4%]\" %5% > /tmp/target_object_info.txt") %
        point_cloud_path % reference_point.x % reference_point.y % reference_point.z % display_options;

    ROS_INFO_STREAM("Command: " << command);

    std::system(command.str().c_str());

    std::ifstream file("/tmp/target_object_info.txt");

    for (std::string line; std::getline(file, line);)
    {
      if (line.empty())
        continue;

      if (line.find("target_obj_contact_points") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.edge_point.x;
        line_iss >> res.edge_point.y;
        line_iss >> res.edge_point.z;
      }

      if (line.find("target_obj_align_vecs") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.alignment_vector.x;
        line_iss >> res.alignment_vector.y;
        line_iss >> res.alignment_vector.z;
      }
    }

    ROS_INFO_STREAM("Target Object Info: \n" << res);
    
    return true;
  }

  bool serveDetectToolInfo(skill_transfer::DetectToolInfo::Request &req,
                           skill_transfer::DetectToolInfo::Response &res)
  {
    const std::string &point_cloud_file_name = req.point_cloud_file_name;
    const std::string point_cloud_path = point_cloud_directory_path_ + point_cloud_file_name;

    const std::string trained_data_file_name = req.task_name + ".mat";
    const std::string trained_data_path = trained_data_directory_path_ + trained_data_file_name;

    std::string display_options =  show_results_ ? "1 1" : "";

    const auto command =
        boost::format("run_get_tool_info.sh /usr/local/MATLAB/MATLAB_Runtime/v93 %1% %2% \"[%3%; %4%; %5%]\" \"[%6% %7% %8%]\" %9% %10% %11% > /tmp/tool_info.txt") %
        point_cloud_path % 
        req.tool_mass % 
        req.alignment_vector.x % 
        req.alignment_vector.y % 
        req.alignment_vector.z %
        req.edge_point.x % 
        req.edge_point.y % 
        req.edge_point.z % 
        req.task_name % 
        trained_data_path %
        display_options;

    ROS_INFO_STREAM("Command: " << command);

    std::system(command.str().c_str());

    std::ifstream file("/tmp/tool_info.txt");

    geometry_msgs::Point point;
    geometry_msgs::Vector3 vector;

    for (std::string line; std::getline(file, line);)
    {
      ROS_INFO_STREAM(line);

      if (line.empty())
        continue;

      if (line.find("affordance_score") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read number
        line_iss >> res.affordance_score;
      }

      if (line.find("grasp_center") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.grasp_center.x;
        line_iss >> res.grasp_center.y;
        line_iss >> res.grasp_center.z;
      }

      if (line.find("action_center") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.action_center.x;
        line_iss >> res.action_center.y;
        line_iss >> res.action_center.z;
      }

      if (line.find("tool_tip") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.tool_tip.x;
        line_iss >> res.tool_tip.y;
        line_iss >> res.tool_tip.z;
      }

      if (line.find("tool_tip_vector") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.tool_tip_vector.x;
        line_iss >> res.tool_tip_vector.y;
        line_iss >> res.tool_tip_vector.z;
      }

      if (line.find("tool_quaternion") != std::string::npos)
      {
        std::getline(file, line);
        std::istringstream line_iss(line);

        // read point
        line_iss >> res.tool_quaternion.x;
        line_iss >> res.tool_quaternion.y;
        line_iss >> res.tool_quaternion.z;
        line_iss >> res.tool_quaternion.w;
      }
    }

    ROS_INFO_STREAM("Tool Info: \n" << res);

    return true;
  }

private:
  geometry_msgs::Vector3 findReferencePoint(std::string object, std::string reference)
  {
    std::string object_frame = name2frame_[object];
    std::string reference_frame = name2frame_[reference];

    geometry_msgs::TransformStamped transform_stamped;

    try
    {
      transform_stamped = tfBuffer.lookupTransform(
          object_frame, reference_frame, ros::Time(0), ros::Duration(10.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("Reference point lookup failed");
      throw;
    }

    return transform_stamped.transform.translation;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_detector");

  FeatureDetector detector;
  ros::spin();

  return 0;
}
