#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <utility>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <skill_transfer/StopCondition.h>
#include <skill_transfer/GetTaskSpec.h>
#include <skill_transfer/GetMotionSpec.h>
#include <skill_transfer/DetectTargetObjectInfo.h>
#include <skill_transfer/DetectToolInfo.h>

class KnowledgeManager
{
private:
  // Possible internal states of the node
  enum State
  {
    Created,
    Initialized,
    Waiting,
    ProcessingKnowledge,
    Ready
  };
  // State
  State state_ = State::Created;
  // ROS handles
  ros::NodeHandle node_handle_;
  ros::ServiceClient target_object_info_service_client_;
  ros::ServiceClient tool_info_service_client_;
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
    target_object_info_service_client_ =
        node_handle_.serviceClient<skill_transfer::DetectTargetObjectInfo>("/feature_detector/detect_target_object_info");

    tool_info_service_client_ =
        node_handle_.serviceClient<skill_transfer::DetectToolInfo>("/feature_detector/detect_tool_info");

    state_ = State::Initialized;
  }

  void start()
  {
    ROS_ASSERT(state_ == State::Initialized);

    state_ = State::Waiting;

    target_object_info_service_client_.waitForExistence();
    tool_info_service_client_.waitForExistence();

    state_ = State::ProcessingKnowledge;

    // Broadcast grasps on TF
    broadcastGrasps();

    // Requesting info from detector

    // When object info is given in setup file skip calling feature_detector
    // This is to enable faster testing and debugging
    if (!setup_["object-info"])
    {
      // Target object info
      if (task_["required-object-info"]["target-object"].as<bool>())
      {
        callDetectTargetObjectInfo();
      }

      // Tool info
      if (task_["required-object-info"]["tool"].as<bool>())
      {
        callDetectToolInfo();
      }
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

    // ROS_INFO_STREAM(res.spec);

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
  /**
   * Makes a service call to feature_detector and saves returned values.
   */
  void callDetectTargetObjectInfo()
  {
    skill_transfer::DetectTargetObjectInfo srv;

    srv.request.point_cloud_file_name =
        setup_["point-clouds"]["target-object"].as<std::string>();

    if (!target_object_info_service_client_.call(srv))
    {
      throw std::runtime_error("Failed to call service detect_target_object_info");
    }

    YAML::Node point_node;
    point_node["vector3"].push_back(srv.response.edge_point.x);
    point_node["vector3"].push_back(srv.response.edge_point.y);
    point_node["vector3"].push_back(srv.response.edge_point.z);

    setup_["object-info"]["edge-point"] = point_node;

    YAML::Node vector_node;
    vector_node["vector3"].push_back(srv.response.alignment_vector.x);
    vector_node["vector3"].push_back(srv.response.alignment_vector.y);
    vector_node["vector3"].push_back(srv.response.alignment_vector.z);

    setup_["object-info"]["alignment-vector"] = vector_node;
  }

  void callDetectToolInfo()
  {
    skill_transfer::DetectToolInfo srv;

    srv.request.point_cloud_file_name =
        setup_["point-clouds"]["tool"].as<std::string>();

    srv.request.task_name = task_["required-object-info"]["task"].as<std::string>();

    srv.request.tool_mass = setup_["tool-mass"].as<double>();

    srv.request.edge_point.x = setup_["object-info"]["edge-point"]["vector3"][0].as<double>();
    srv.request.edge_point.y = setup_["object-info"]["edge-point"]["vector3"][1].as<double>();
    srv.request.edge_point.z = setup_["object-info"]["edge-point"]["vector3"][2].as<double>();

    srv.request.alignment_vector.x = setup_["object-info"]["alignment-vector"]["vector3"][0].as<double>();
    srv.request.alignment_vector.y = setup_["object-info"]["alignment-vector"]["vector3"][1].as<double>();
    srv.request.alignment_vector.z = setup_["object-info"]["alignment-vector"]["vector3"][2].as<double>();

    if (!tool_info_service_client_.call(srv))
    {
      throw std::runtime_error("Failed to call service detect_target_object_info");
    }

    YAML::Node grasp_node;
    grasp_node["vector3"].push_back(srv.response.grasp_center.x);
    grasp_node["vector3"].push_back(srv.response.grasp_center.y);
    grasp_node["vector3"].push_back(srv.response.grasp_center.z);

    setup_["object-info"]["grasp-center"] = grasp_node;

    YAML::Node center_node;
    center_node["vector3"].push_back(srv.response.action_center.x);
    center_node["vector3"].push_back(srv.response.action_center.y);
    center_node["vector3"].push_back(srv.response.action_center.z);

    setup_["object-info"]["action-center"] = center_node;

    YAML::Node tip_node;
    tip_node["vector3"].push_back(srv.response.tool_tip.x);
    tip_node["vector3"].push_back(srv.response.tool_tip.y);
    tip_node["vector3"].push_back(srv.response.tool_tip.z);

    setup_["object-info"]["tool-tip"] = tip_node;

    YAML::Node tip_vector_node;
    tip_vector_node["vector3"].push_back(srv.response.tool_tip_vector.x);
    tip_vector_node["vector3"].push_back(srv.response.tool_tip_vector.y);
    tip_vector_node["vector3"].push_back(srv.response.tool_tip_vector.z);

    setup_["object-info"]["tool-tip-vector"] = tip_vector_node;

    YAML::Node orientation_node;
    orientation_node["quaternion"].push_back(srv.response.tool_quaternion.x);
    orientation_node["quaternion"].push_back(srv.response.tool_quaternion.y);
    orientation_node["quaternion"].push_back(srv.response.tool_quaternion.z);
    orientation_node["quaternion"].push_back(srv.response.tool_quaternion.w);

    setup_["object-info"]["tool-quaternion"] = orientation_node;

    YAML::Node heel_node;
    heel_node["vector3"].push_back(srv.response.tool_heel.x);
    heel_node["vector3"].push_back(srv.response.tool_heel.y);
    heel_node["vector3"].push_back(srv.response.tool_heel.z);

    setup_["object-info"]["tool-heel"] = heel_node;
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
    const YAML::Node &all_features_node = setup_["object-info"];

    for (YAML::const_iterator it = all_features_node.begin(); it != all_features_node.end(); ++it)
    {
      YAML::Node fn;
      fn[it->first] = it->second;

      new_scope.push_back(fn);
    }

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

    try
    {
      msg.measured_velocity_min = node["measured-velocity-min-threshold"].as<double>();
      msg.desired_velocity_min = node["desired-velocity-min-threshold"].as<double>();
      msg.contact = node["contact"].as<bool>();
      msg.activation_distance = node["activation-distance"].as<double>();
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Failed to parse stop condition");
      throw;
    }

    return msg;
  }

  void broadcastGrasps()
  {
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