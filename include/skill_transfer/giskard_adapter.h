#ifndef GISKARD_ADAPTER_H
#define GISKARD_ADAPTER_H

#include <giskard_core/giskard_core.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>

class GiskardAdapter
{
public:
  GiskardAdapter(int nWSR);
  
  void createController(const std::string &constraints);
  void startController(const Eigen::VectorXd &inputs);
  void updateController(const Eigen::VectorXd &inputs);
  geometry_msgs::Twist getDesiredFrameTwistMsg(
                                      const Eigen::VectorXd &inputs,
                                      const std::string &frame_name);
  sensor_msgs::JointState getDesiredJointVelocityMsg();
  double getDistance();
  std::vector<visualization_msgs::Marker> getVisualizationMsgs();
  
  bool controller_started_;
  int nWSR_;
  
private:
  giskard_core::QPController controller_;
};

#endif // GISKARD_ADAPTER_H
