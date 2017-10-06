#ifndef TASK_H
#define TASK_H

#include <vector>
#include <string>

struct SceneObjects {
  std::string gripper_link_name;
  std::string tool_link_name;
  std::string utility_link_name;
};

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
  Task(std::string file_path);
  
  std::string name;
  SceneObjects scene_objects;
  std::vector<MotionPhase> phases;
  
  bool hasNextPhase();
  std::string getCurrentPhaseSpec();
  MotionPhase getCurrentPhase();
  void completeCurrentPhase();
  
private:
  unsigned int current_phase_index_;
};

#endif // TASK_H
