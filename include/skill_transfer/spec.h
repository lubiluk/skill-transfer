#ifndef SPEC_H
#define SPEC_H

#include "skill_transfer/ObjectFeature.h"

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

struct StopCondition {
  double measured_velocity_min;
  double desired_velocity_min;
  bool contact;
  double activation_distance;
};

class Spec
{
public:
  Spec(std::string setup_file_path, std::string task_file_path, std::string motion_template_file_path);
  
  std::vector<skill_transfer::ObjectFeature> getRequiredObjectFeatures();
  void setObjectFeatures(std::vector<skill_transfer::ObjectFeature> features);
  
  std::size_t getMotionCount();
  std::string getMotionSpec(std::size_t index);
  StopCondition getMotionStopCondition(std::size_t index);

private:
  YAML::Node setup_;
  YAML::Node task_;
  YAML::Node motion_template_;
};

#endif // SPEC_H
