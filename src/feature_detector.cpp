#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <boost/format.hpp>
#include <fstream>
#include <map>

#include <skill_transfer/DetectObjectFeature.h>

class FeatureDetector
{
private:
  // ROS handles
  ros::NodeHandle node_handle_;
  ros::ServiceServer detector_service_server_;
  // File directories
  std::string point_cloud_directory_path_;
  // TF
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::map<std::string, std::string> name2frame_;
  // Additional parameters
  bool print_output_ = false;
  bool plot_results_ = false;

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

    node_handle_.getParam("print_output", print_output_);
    node_handle_.getParam("plot_results", plot_results_);

    // Start services
    detector_service_server_ =
        node_handle_.advertiseService("detect_object_feature",
                                      &FeatureDetector::serveDetectObjectFeature,
                                      this);
  }

  bool serveDetectObjectFeature(skill_transfer::DetectObjectFeature::Request &req,
                                skill_transfer::DetectObjectFeature::Response &res)
  {
    // Facade
    std::string feature = req.object_feature.feature;

    if (feature == "edge-point")
    {
      serveDetectEdgePoint(req, res);
    }
    else
    {
      ROS_ERROR_STREAM("Unknown feature requested: " << feature);
      return false;
    }

    return true;
  }

private:
  void serveDetectEdgePoint(skill_transfer::DetectObjectFeature::Request &req,
                            skill_transfer::DetectObjectFeature::Response &res)
  {
    // Find reference point
    const geometry_msgs::Vector3 reference_point =
        findReferencePoint(req.object_feature.object, req.object_feature.reference);

    const std::string &point_cloud_file_name = req.point_cloud_file_name;
    const std::string point_cloud_path = point_cloud_directory_path_ + point_cloud_file_name;

    ROS_INFO_STREAM("Reference point: "
                    << reference_point.x << " "
                    << reference_point.y << " "
                    << reference_point.z);

    const auto command =
        boost::format("run_edge_detector.sh /usr/local/MATLAB/MATLAB_Runtime/v92 %1% \"[%2% %3% %4%]\" %5% %6% > edge.txt") %
        point_cloud_path % reference_point.x % reference_point.y % reference_point.z % print_output_ % plot_results_;

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

    res.object_feature.value = point;

    ROS_INFO_STREAM("Edge point: " << point.x << " " << point.y << " " << point.z);
  }

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
