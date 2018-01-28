#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <utility>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <skill_transfer/ObjectFeature.h>
#include <skill_transfer/StopCondition.h>
#include <skill_transfer/GetTaskSpec.h>
#include <skill_transfer/GetMotionSpec.h>
#include <skill_transfer/DetectObjectFeature.h>

class KnowledgeManager
{
private:
  // Possible internal states of the node
  enum State
  {
    Created,
    Initialized,
    ProcessingKnowledge,
    Ready
  };
  // State
  State state_ = State::Created;
  // ROS handles
  ros::NodeHandle node_handle_;
  ros::ServiceClient feature_service_client_;
  ros::ServiceServer task_spec_service_server_;
  ros::ServiceServer motion_spec_service_server_;
  // File paths
  std::string task_file_path_;
  std::string setup_file_path_;
  std::string motion_template_file_path_;
  // File directories
  std::string motion_directory_path_;
  // YAML files
  YAML::Node setup_;
  YAML::Node task_;
  YAML::Node motion_template_;
  // TF2
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;

public:
  KnowledgeManager() : node_handle_("~")
  {
    // Load values from ROSParam

    if (!node_handle_.getParam("task_file_path", task_file_path_))
    {
      throw std::runtime_error("Could not find parameter 'task_file_path' in namespace '" +
                               node_handle_.getNamespace() + "'.");
    }

    if (!node_handle_.getParam("setup_file_path", setup_file_path_))
    {
      throw std::runtime_error("Could not find parameter 'setup_file_path' in namespace '" +
                               node_handle_.getNamespace() + "'.");
    }

    if (!node_handle_.getParam("motion_template_file_path", motion_template_file_path_))
    {
      throw std::runtime_error("Could not find parameter 'motion_template_file_path' in namespace '" +
                               node_handle_.getNamespace() + "'.");
    }

    if (!node_handle_.getParam("motion_directory_path", motion_directory_path_))
    {
      throw std::runtime_error("Could not find parameter 'motion_directory_path' in namespace '" +
                               node_handle_.getNamespace() + "'.");
    }

    // Load files
    try
    {
      setup_ = YAML::LoadFile(setup_file_path_);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Could not load setup file");
      throw;
    }

    try
    {
      task_ = YAML::LoadFile(task_file_path_);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Could not load task file");
      throw;
    }

    try
    {
      motion_template_ = YAML::LoadFile(motion_template_file_path_);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Could not load motion template file");
      throw;
    }

    // Initialize servers and clients
    feature_service_client_ =
        node_handle_.serviceClient<skill_transfer::DetectObjectFeature>("detect_object_feature");

    state_ = State::Initialized;
  }

  void start()
  {
    ROS_ASSERT(state_ == State::Initialized);

    state_ = State::ProcessingKnowledge;

    // Broadcast grasps on TF
    {
      const auto &tool_grasp_frame = setup_["target-object-grasp"]["frame"];
      double qx, qy, qz, qw, x, y, z;

      for (const auto &n : tool_grasp_frame)
      {
        if (n["quaternion"])
        {
          const auto &q = n["quaternion"];

          qx = q[0].as<double>();
          qy = q[1].as<double>();
          qz = q[2].as<double>();
          qw = q[3].as<double>();
        }

        if (n["vector3"])
        {
          const auto &v = n["vector3"];

          x = v[0].as<double>();
          y = v[1].as<double>();
          z = v[2].as<double>();
        }
      }

      geometry_msgs::TransformStamped transform_stamped;

      transform_stamped.header.frame_id = "r_gripper_tool_frame";
      transform_stamped.child_frame_id = "target_object_frame";
      transform_stamped.header.stamp = ros::Time::now();

      transform_stamped.transform.translation.x = x;
      transform_stamped.transform.translation.y = y;
      transform_stamped.transform.translation.z = z;
      transform_stamped.transform.rotation.x = qx;
      transform_stamped.transform.rotation.y = qy;
      transform_stamped.transform.rotation.z = qz;
      transform_stamped.transform.rotation.w = qw;

      tf_broadcaster_.sendTransform(transform_stamped);
    }
    {
      const auto &tool_grasp_frame = setup_["tool-grasp"]["frame"];
      double qx, qy, qz, qw, x, y, z;

      for (const auto &n : tool_grasp_frame)
      {
        if (n["quaternion"])
        {
          const auto &q = n["quaternion"];

          qx = q[0].as<double>();
          qy = q[1].as<double>();
          qz = q[2].as<double>();
          qw = q[3].as<double>();
        }

        if (n["vector3"])
        {
          const auto &v = n["vector3"];

          x = v[0].as<double>();
          y = v[1].as<double>();
          z = v[2].as<double>();
        }
      }

      geometry_msgs::TransformStamped transform_stamped;

      transform_stamped.header.frame_id = "l_gripper_tool_frame";
      transform_stamped.child_frame_id = "tool_frame";
      transform_stamped.header.stamp = ros::Time::now();

      transform_stamped.transform.translation.x = x;
      transform_stamped.transform.translation.y = y;
      transform_stamped.transform.translation.z = z;
      transform_stamped.transform.rotation.x = qx;
      transform_stamped.transform.rotation.y = qy;
      transform_stamped.transform.rotation.z = qz;
      transform_stamped.transform.rotation.w = qw;

      tf_broadcaster_.sendTransform(transform_stamped);
    }

    // Requesting features from detector
    std::vector<std::pair<std::string, skill_transfer::ObjectFeature>>
        required_features = getRequiredObjectFeatures();

    for (const auto &rf : required_features)
    {
      skill_transfer::ObjectFeature feature = callDetectObjectFeature(rf.second);
      setObjectFeature(rf.first, feature);
    }

    // Starting services
    task_spec_service_server_ =
        node_handle_.advertiseService("get_task_spec",
                                      &KnowledgeManager::serveGetTaskSpec,
                                      this);
    motion_spec_service_server_ =
        node_handle_.advertiseService("get_motion_spec",
                                      &KnowledgeManager::serveGetMotionSpec,
                                      this);

    state_ = State::Ready;
  }

  bool serveGetMotionSpec(skill_transfer::GetMotionSpec::Request &req,
                          skill_transfer::GetMotionSpec::Response &res)
  {
    ROS_ASSERT(state_ == State::Ready);

    std::size_t index = req.index; // implicit type conversion
    res.stop_condition = getMotionStopCondition(index);
    res.spec = getMotionSpec(index);

    return true;
  }

  bool serveGetTaskSpec(skill_transfer::GetTaskSpec::Request &req,
                        skill_transfer::GetTaskSpec::Response &res)
  {
    ROS_ASSERT(state_ == State::Ready);

    res.motion_phase_count = getMotionCount(); // implicit type conversion

    return true;
  }

private:
  std::vector<std::pair<std::string, skill_transfer::ObjectFeature>> getRequiredObjectFeatures() const
  {
    const YAML::Node &rofn = task_["required-object-features"];
    std::vector<std::pair<std::string, skill_transfer::ObjectFeature>> ofv;

    for (YAML::const_iterator oit = rofn.begin(); oit != rofn.end(); ++oit)
    {
      const std::string feature_name = oit->first.as<std::string>();
      const YAML::Node &fn = oit->second;

      skill_transfer::ObjectFeature of;
      of.object = fn["object"].as<std::string>();
      of.feature = fn["feature"].as<std::string>();
      of.reference = fn["reference"].as<std::string>();

      ofv.push_back(std::make_pair(feature_name, of));
    }

    return ofv;
  }

  void setObjectFeature(std::string name, skill_transfer::ObjectFeature feature)
  {
    YAML::Node point_node;

    point_node["vector3"].push_back(feature.value.x);
    point_node["vector3"].push_back(feature.value.y);
    point_node["vector3"].push_back(feature.value.z);

    setup_["object-features"][name] = point_node;
  }

  /**
   * Makes a service call to feature_detector and returns a filled in object.
   */
  skill_transfer::ObjectFeature callDetectObjectFeature(skill_transfer::ObjectFeature feature)
  {
    skill_transfer::DetectObjectFeature srv;
    srv.request.object_feature = feature;

    srv.request.point_cloud_file_name =
        setup_["point-clouds"][feature.object].as<std::string>();

    if (!feature_service_client_.call(srv))
    {
      throw std::runtime_error("Failed to call service detect_object_feature");
    }

    return srv.response.object_feature;
  }

  std::size_t getMotionCount() const
  {
    return task_["motion-phases"].size();
  }

  /** 
 * Reads motion YAML file, combines it with 
 * motion template YAML file and
 * fills in the gaps, i. e. grasps, object features.
 * Returns the spec as a string.
 * 
 * @return string Complete motion phase spec.
*/
  std::string getMotionSpec(std::size_t index) const
  {
    ROS_ASSERT(index >= 0 && index < task_["motion-phases"].size());

    YAML::Node phase = task_["motion-phases"][index];

    // Read the motion phase file
    boost::filesystem::path dir_path(motion_directory_path_);
    std::string file_path = phase["file"].as<std::string>();
    const boost::filesystem::path path = dir_path / file_path;

    if (!boost::filesystem::exists(path))
    {
      throw std::runtime_error("File not found: " + path.string());
    }

    const YAML::Node phase_spec = YAML::LoadFile(path.string());
    YAML::Node motion_spec = YAML::Clone(motion_template_);

    // Merge the template and the motion spec
    const YAML::Node motion_spec_scope = motion_spec["scope"];
    const YAML::Node scope = phase_spec["scope"];
    const YAML::Node constraints = phase_spec["soft-constraints"];

    // Fill in grasps
    // They have to be put in front of the scope, so we
    // make a new scope and re-add things
    YAML::Node new_scope;

    YAML::Node tool_grasp_node;
    tool_grasp_node["tool-grasp"] = setup_["tool-grasp"];
    YAML::Node target_object_grasp_node;
    target_object_grasp_node["target-object-grasp"] = setup_["target-object-grasp"];
    new_scope.push_back(tool_grasp_node);
    new_scope.push_back(target_object_grasp_node);

    // Fill in object features
    // new_scope.push_back(setup_["object-features"]);

    // Fill in template scope
    for (YAML::const_iterator it = motion_spec_scope.begin(); it != motion_spec_scope.end(); ++it)
    {
      new_scope.push_back(*it);
    }

    // Fill in the phase scope
    for (YAML::const_iterator it = scope.begin(); it != scope.end(); ++it)
    {
      new_scope.push_back(*it);
    }

    // Replace scope
    motion_spec["scope"] = new_scope;
    // Insert constraints
    motion_spec["soft-constraints"] = constraints;

    // Convert spec to string
    YAML::Emitter out;
    out << motion_spec;
    std::string spec{out.c_str()};

    return spec;
  }

  skill_transfer::StopCondition getMotionStopCondition(std::size_t index) const
  {
    ROS_ASSERT(index >= 0 && index < task_["motion-phases"].size());

    const YAML::Node &node = task_["motion-phases"][index]["stop"];
    skill_transfer::StopCondition msg;

    msg.measured_velocity_min = node["measured-velocity-min-threshold"].as<double>();
    msg.desired_velocity_min = node["desired-velocity-min-threshold"].as<double>();
    msg.contact = node["contact"].as<bool>();
    msg.activation_distance = node["activation-distance"].as<double>();

    return msg;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowledge_manager");
  KnowledgeManager manager;
  manager.start();
  ros::spin();

  return 0;
}