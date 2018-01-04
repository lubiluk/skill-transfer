#include "skill_transfer/task.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <algorithm>


Task::Task() : current_phase_index_(0)
{

}

void Task::load(std::string task_file_path, std::string motion_template_file_path)
{
  const YAML::Node spec = YAML::LoadFile(task_file_path);
  this->motion_template_ = YAML::LoadFile(motion_template_file_path);
   
  this->name = spec["name"].as<std::string>();
  
  const auto &objectSpecs = spec["objects"];
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

YAML::Node Task::getCurrentPhaseSpecNode()
{
  const auto &phase = phases[current_phase_index_];
  boost::filesystem::path dir_path(motion_directory_path);
  const auto path = dir_path / phase.file_path;
  
  if (!boost::filesystem::exists(path)) {
    throw std::runtime_error("File not found: " + path.string());
  }
  
  const YAML::Node phase_spec = YAML::LoadFile(path.string());
  auto motion_spec = YAML::Clone(motion_template_);
  
  // Merge the template and the motion spec
  auto motion_spec_scope = motion_spec["scope"];
  YAML::Node scope = phase_spec["scope"];
  const auto constraints = phase_spec["soft-constraints"];
  
  for(const auto n : scope) {
    motion_spec_scope.push_back(n);
  }
  
  motion_spec["soft-constraints"] = constraints;
  
  return motion_spec;
}

MotionPhase Task::getCurrentPhase()
{
  return phases[current_phase_index_];
}

void Task::completeCurrentPhase()
{
  current_phase_index_ += 1;
}

bool Task::resolveContains(std::string key)
{
  const auto &r = resolve;
    
  return std::find(r.begin(), r.end(), key) != r.end();
}


