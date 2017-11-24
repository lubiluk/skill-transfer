#include "skill_transfer/giskard_adapter.h"
#include "skill_transfer/conversions.h"
#include "skill_transfer/giskard_utils.h"
#include "skill_transfer/giskard_viz.h"

GiskardAdapter::GiskardAdapter(int nWSR): nWSR_(nWSR)
{

}

void GiskardAdapter::createController(const std::string &constraints)
{
  controller_started_ = false;
  controller_ = generateController(constraints);
}

void GiskardAdapter::startController(Eigen::VectorXd inputs)
{
  if (!controller_started_)
  {
    if (!controller_.start(inputs, nWSR_))
    {
      throw std::runtime_error("Failed to start controller");
    }
    
    controller_started_ = true;
  }
  else
  {
    ROS_WARN("GiskardAdapter: Attempt to start an active controller");
  }
}

void GiskardAdapter::updateController(Eigen::VectorXd inputs)
{
  if (!controller_.update(inputs, nWSR_))
  {
    throw std::runtime_error("Failed to update controller");
  }
}

geometry_msgs::Twist GiskardAdapter::getDesiredFrameTwistMsg(
                                      const Eigen::VectorXd &inputs,
                                      const std::string &frame_name)
{
  const Eigen::VectorXd desired_velocity = 
    getJacobian(controller_, frame_name, inputs).data * controller_.get_command();
                 
  return eigenVectorToMsgTwist(desired_velocity);
}

sensor_msgs::JointState GiskardAdapter::getDesiredJointVelocityMsg()
{
  return eigenVectorToMsgJointState(controller_.get_command());
}

double GiskardAdapter::getDistance()
{
  const KDL::Expression<KDL::Vector>::Ptr distance_exp =
    controller_.get_scope().find_vector_expression("distance");
  auto distance_vector = distance_exp->value();
  double distance = distance_vector.Norm();
  
  return distance;
}

std::vector<visualization_msgs::Marker> GiskardAdapter::getVisualizationMsgs()
{
  return std::vector<visualization_msgs::Marker> {
    createPointMarker(controller_, "tool-point", "base_footprint"),
    createPointMarker(controller_, "target-object-point", "base_footprint"),
    createPointDirectionMarker(controller_, "tool-point", "distance", "base_footprint")
  };
}

