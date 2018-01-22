#include "skill_transfer/spec.h"

namespace YAML {
template<>
struct convert<::skill_transfer::ObjectFeature>;
struct convert<StopCondition>;
};

Spec::Spec(std::string setup_file_path,
           std::string task_file_path,
           std::string motion_template_file_path)
{
  // Load setup file
  setup_ = YAML::LoadFile(setup_file_path);

  // Load task file
  task_ = YAML::LoadFile(task_file_path);

  // Load motion template file
  motion_template_ = YAML::LoadFile(motion_template_file_path);
}

std::vector<skill_transfer::ObjectFeature> Spec::getRequiredObjectFeatures()
{
  return task_["required-object-features"].as<skill_transfer::ObjectFeature>();
}

void Spec::setObjectFeatures(std::vector<skill_transfer::ObjectFeature> features)
{
  setup_["object-features"] = features;
}

std::size_t Spec::getMotionCount()
{
  return task_["motion-phases"].size();
}

std::string getMotionSpec(std::size_t index)
{
  // Fill in the gaps and return string
}

StopCondition Spec::getMotionStopCondition(std::size_t index)
{
  return task_["motion-phases"][index]["stop"].as<StopCondition>();
}

namespace YAML {

template<>
struct convert<::skill_transfer::ObjectFeature> {
  static Node encode(const ::skill_transfer::ObjectFeature& rhs) {
    Node node;
    node["object"] = rhs.object;
    node["feature"] = rhs.feature;
    node["point"]["vector3"].push_back(rhs.point.x);
    node["point"]["vector3"].push_back(rhs.point.y);
    node["point"]["vector3"].push_back(rhs.point.z);

    return node;
  }

  static bool decode(const Node& node, ::skill_transfer::ObjectFeature& rhs) {
    rhs.feature = node["feature"].as<std::string>();
    rhs.object = node["object"].as<std::string>();
    rhs.point.x = node["point"]["vector3"][0].as<double>();
    rhs.point.y = node["point"]["vector3"][1].as<double>();
    rhs.point.z = node["point"]["vector3"][2].as<double>();

    return true;
  }
};

template<>
struct convert<StopCondition> {
  static Node encode(const StopCondition& rhs) {
    Node node;
    node["measured-velocity-min-threshold"] = rhs.measured_velocity_min;
    node["desired-velocity-min-threshold"] = rhs.desired_velocity_min;
    node["contact"] = rhs.contact;
    node["activation-distance"] = rhs.activation_distance;

    return node;
  }

  static bool decode(const Node& node, StopCondition& rhs) {
    rhs.measured_velocity_min = node["measured-velocity-min-threshold"].as<double>();
    rhs.desired_velocity_min = node["desired-velocity-min-threshold"].as<double>();
    rhs.contact = node["contact"].as<bool>();
    rhs.activation_distance = node["activation-distance"].as<double>();

    return true;
  }
};

}