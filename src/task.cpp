#include "skill_transfer/task.h"

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iostream>

Task::Task() : current_phase_index_(0)
{

}

void Task::load(std::string file_name)
{
  YAML::Node spec = YAML::LoadFile(file_name);
  
  this->name = spec["name"].as<std::string>();
  
  const auto &objectSpecs = spec["objects"];
  
  this->scene_objects.gripper_link_name = objectSpecs["gripper-link-name"].as<std::string>();
  this->scene_objects.tool_link_name = objectSpecs["tool-link-name"].as<std::string>();
  this->scene_objects.utility_link_name = objectSpecs["utility-link-name"].as<std::string>();
  
  const auto &phaseSpecs = spec["motion-phases"];
  
  for (std::size_t i=0; i < phaseSpecs.size(); ++i) {
    const auto &phaseSpec = phaseSpecs[i];
  
    MotionPhase phase;
    StopCondition stop_condition;
    
    phase.name = phaseSpec["name"].as<std::string>();
    phase.file_path = phaseSpec["file"].as<std::string>();
    phase.stop_activation_distance = phaseSpec["stop-activation-distance"].as<double>();
    
    const auto &stopSpec = phaseSpec["stop"];
    
    stop_condition.measured_velocity_min = stopSpec["measured-velocity-min-threshold"].as<double>();
    stop_condition.desired_velocity_min = stopSpec["desired-velocity-min-threshold"].as<double>();
    stop_condition.contact = stopSpec["contact"].as<bool>();
    
    phase.stop_condition = stop_condition;
    
    phases.push_back(phase);
  }
}

bool Task::hasNextPhase()
{
  return current_phase_index_ < phases.size();
}

std::string Task::getCurrentPhaseSpec()
{
  const auto &phase = phases[current_phase_index_];
  boost::filesystem::path dir_path(motion_directory_path);
  const auto path = dir_path / phase.file_path;
  
  if (!boost::filesystem::exists(path)) {
    throw std::runtime_error("File not found: " + path.string());
  }
  
  boost::filesystem::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  
  return buffer.str();
}

MotionPhase Task::getCurrentPhase()
{
  return phases[current_phase_index_];
}

void Task::completeCurrentPhase()
{
  current_phase_index_ += 1;
}

