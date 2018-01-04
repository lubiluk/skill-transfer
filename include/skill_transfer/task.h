#ifndef TASK_H
#define TASK_H

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

struct StopCondition {
  double measured_velocity_min;
  double desired_velocity_min;
  bool contact;
};

struct MotionPhase {
  std::string name;
  std::string file_path;
  double stop_activation_distance;
  StopCondition stop_condition;
};

class Task
{
public:
  Task();
  
  void load(std::string task_file_path, std::string motion_template_file_path);
  std::string name;
  std::vector<MotionPhase> phases;
  std::string motion_directory_path;
  std::vector<std::string> resolve;
  
  bool hasNextPhase();
  YAML::Node getCurrentPhaseSpecNode();
  MotionPhase getCurrentPhase();
  void completeCurrentPhase();
  bool resolveContains(std::string key);
  
private:
  unsigned int current_phase_index_;
  YAML::Node motion_template_;
};

#endif // TASK_H
