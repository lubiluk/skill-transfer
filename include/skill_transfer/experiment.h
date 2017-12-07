#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>
#include <string>

class Experiment
{
public:  
  void load(std::string experiment_file_path);
  
  std::string name;
  
  std::string task_file_name;
  
  std::string tool_file_name;
  std::string target_object_file_name;
  
  YAML::Node tool_grasp_node;
  YAML::Node target_object_grasp_node;
  tf2::Transform tool_grasp_transform;
  tf2::Transform target_object_grasp_transform;
  
private:

};

#endif // EXPERIMENT_H
