#include "skill_transfer/experiment.h"

void Experiment::load(std::string experiment_file_path)
{
  const YAML::Node spec = YAML::LoadFile(experiment_file_path);
   
  this->name = spec["name"].as<std::string>();
  this->task_file_name = spec["task"].as<std::string>();
  this->tool_file_name = spec["task"].as<std::string>();
  this->target_object_file_name = spec["task"].as<std::string>();
  
  this->tool_grasp_node = spec["tool-grasp"];
  this->target_object_grasp_node = spec["target-object-grasp"];
}
