#include "skill_transfer/spec.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace YAML
{
template <>
struct convert<StopCondition>;
};

Spec::Spec(std::string setup_file_path,
           std::string task_file_path,
           std::string motion_template_file_path,
           std::string motion_directory_path)
    : motion_directory_path(motion_directory_path)
{
  // Load setup file
  setup_ = YAML::LoadFile(setup_file_path);

  // Load task file
  task_ = YAML::LoadFile(task_file_path);

  // Load motion template file
  motion_template_ = YAML::LoadFile(motion_template_file_path);
}

std::vector<skill_transfer::ObjectFeature> Spec::getRequiredObjectFeatures() const
{
  const YAML::Node &rofn = task_["required-object-features"];
  std::vector<skill_transfer::ObjectFeature> ofv;

  for (YAML::const_iterator oit = rofn.begin(); oit != rofn.end(); ++oit)
  {
    const std::string object_name = oit->first.as<std::string>();
    const YAML::Node &fn = oit->second;

    for (YAML::const_iterator fit = rofn.begin(); fit != fn.end(); ++fit)
    {
      skill_transfer::ObjectFeature of;
      of.object = object_name;
      of.feature = fit->as<std::string>();

      ofv.push_back(of);
    }
  }

  return ofv;
}

void Spec::setObjectFeature(skill_transfer::ObjectFeature feature)
{
  YAML::Node point_node;

  point_node["vector3"].push_back(feature.value.x);
  point_node["vector3"].push_back(feature.value.y);
  point_node["vector3"].push_back(feature.value.z);

  setup_["object-features"][feature.object][feature.feature] = point_node;
}

std::size_t Spec::getMotionCount() const
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
std::string Spec::getMotionSpec(std::size_t index) const
{
  assert(index > 0 && index < task_["motion-phases"].size());

  YAML::Node phase = task_["motion-phases"][index];

  // Read the motion phase file
  boost::filesystem::path dir_path(motion_directory_path);
  std::string file_path = phase["file"].as<std::string>();
  const boost::filesystem::path path = dir_path / file_path;

  if (!boost::filesystem::exists(path))
  {
    throw std::runtime_error("File not found: " + path.string());
  }

  const YAML::Node phase_spec = YAML::LoadFile(path.string());
  YAML::Node motion_spec = YAML::Clone(motion_template_);

  // Merge the template and the motion spec
  YAML::Node motion_spec_scope = motion_spec["scope"];
  YAML::Node scope = phase_spec["scope"];
  const YAML::Node constraints = phase_spec["soft-constraints"];

  for (const auto n : scope)
  {
    motion_spec_scope.push_back(n);
  }

  motion_spec["soft-constraints"] = constraints;

  // Fill in grasps
  // They have to be put in front of the scope, so we
  // make a new scope and re-add things
  YAML::Node new_scope;
  new_scope.push_back(setup_["tool-grasp"]);
  new_scope.push_back(setup_["target-object-grasp"]);

  // Fill in object features
  new_scope.push_back(setup_["object-features"]);

  // Fill in the rest of the scope
  for (YAML::const_iterator it = scope.begin(); it != scope.end(); ++it)
  {
    new_scope.push_back(*it);
  }

  // Replace scope
  motion_spec["scope"] = new_scope;

  // Convert spec to string
  YAML::Emitter out;
  out << motion_spec;
  std::string spec{out.c_str()};

  return spec;
}

StopCondition Spec::getMotionStopCondition(std::size_t index) const
{
  return task_["motion-phases"][index]["stop"].as<StopCondition>();
}

namespace YAML
{
template <>
struct convert<StopCondition>
{
  static Node encode(const StopCondition &rhs)
  {
    Node node;
    node["measured-velocity-min-threshold"] = rhs.measured_velocity_min;
    node["desired-velocity-min-threshold"] = rhs.desired_velocity_min;
    node["contact"] = rhs.contact;
    node["activation-distance"] = rhs.activation_distance;

    return node;
  }

  static bool decode(const Node &node, StopCondition &rhs)
  {
    rhs.measured_velocity_min = node["measured-velocity-min-threshold"].as<double>();
    rhs.desired_velocity_min = node["desired-velocity-min-threshold"].as<double>();
    rhs.contact = node["contact"].as<bool>();
    rhs.activation_distance = node["activation-distance"].as<double>();

    return true;
  }
};
}
