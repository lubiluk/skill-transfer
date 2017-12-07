#include "skill_transfer/experiment.h"

void Experiment::load(std::string experiment_file_path)
{
  const YAML::Node spec = YAML::LoadFile(experiment_file_path);
   
  this->name = spec["name"].as<std::string>();
  this->task_file_name = spec["task"].as<std::string>();
  this->tool_file_name = spec["task"].as<std::string>();
  this->target_object_file_name = spec["target-object-3d-scan"].as<std::string>();
  
  this->tool_grasp_node = spec["tool-grasp"];
  this->target_object_grasp_node = spec["target-object-grasp"];
  
  {
    const auto &tool_grasp_frame = spec["tool-grasp"]["frame"];
  
    double qx, qy, qz, qw, x, y, z;
    
    for (const auto &n: tool_grasp_frame) {
      if (n["quaternion"]) {
        const auto &q = n["quaternion"];
        
        qx = q[0].as<double>();
        qy = q[1].as<double>();
        qz = q[2].as<double>();
        qw = q[3].as<double>();
      }
      
      if (n["vector3"]) {
        const auto &v = n["vector3"];
        
        x = v[0].as<double>();
        y = v[1].as<double>();
        z = v[2].as<double>();
      }
    }
    
    const auto quat = tf2::Quaternion(qx, qy, qz, qw);
    const auto vec = tf2::Vector3(x, y, z);
    
    this->tool_grasp_transform = tf2::Transform(quat, vec);
  }
  
  {
    const auto &tool_grasp_frame = spec["target-object-grasp"]["frame"];
  
    double qx, qy, qz, qw, x, y, z;
    
    for (const auto &n: tool_grasp_frame) {
      if (n["quaternion"]) {
        const auto &q = n["quaternion"];
        
        qx = q[0].as<double>();
        qy = q[1].as<double>();
        qz = q[2].as<double>();
        qw = q[3].as<double>();
      }
      
      if (n["vector3"]) {
        const auto &v = n["vector3"];
        
        x = v[0].as<double>();
        y = v[1].as<double>();
        z = v[2].as<double>();
      }
    }
    
    const auto quat = tf2::Quaternion(qx, qy, qz, qw);
    const auto vec = tf2::Vector3(x, y, z);
    
    this->target_object_grasp_transform = tf2::Transform(quat, vec);
  }
  
}
