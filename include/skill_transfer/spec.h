#ifndef SPEC_H
#define SPEC_H

#include "skill_transfer/ObjectFeature.h"
#include "skill_transfer/stop_condition.h"

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

class Spec
{
public:
  std::string motion_directory_path;

public:
  Spec(std::string setup_file_path,
       std::string task_file_path,
       std::string motion_template_file_path,
       std::string motion_directory_path);

  std::vector<skill_transfer::ObjectFeature> getRequiredObjectFeatures() const;
  void setObjectFeature(skill_transfer::ObjectFeature feature);

  std::size_t getMotionCount() const;
  std::string getMotionSpec(std::size_t index) const;
  StopCondition getMotionStopCondition(std::size_t index) const;

private:
  YAML::Node setup_;
  YAML::Node task_;
  YAML::Node motion_template_;
};

#endif // SPEC_H
