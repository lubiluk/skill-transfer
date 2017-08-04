#include <visualization_msgs/Marker.h>
#include <giskard_core/giskard_core.hpp>

inline visualization_msgs::Marker createPointMarker(const giskard_core::QPController &controller,
                                                    const std::string &exp_name, const std::string &frame_id)
{
  const KDL::Expression<KDL::Vector>::Ptr exp =
      controller.get_scope().find_vector_expression(exp_name);

  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "giskard_expressions/" + exp_name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = exp->value().x();
  marker.pose.position.y = exp->value().y();
  marker.pose.position.z = exp->value().z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.r = 244.0f / 255.0f;
  marker.color.g = 180.0f / 255.0f;
  marker.color.b = 47.0f / 255.0f;
  marker.color.a = 1.0;

  return marker;
}

inline visualization_msgs::Marker createPointDirectionMarker(const giskard_core::QPController &controller,
                                                             const std::string &point_name,
                                                             const std::string &direction_name,
                                                             const std::string &frame_id)
{
  const KDL::Expression<KDL::Vector>::Ptr point_exp =
      controller.get_scope().find_vector_expression(point_name);
  const KDL::Expression<KDL::Vector>::Ptr direction_exp =
      controller.get_scope().find_vector_expression(direction_name);

  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "giskard_expressions/" + direction_name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(2);
  marker.points[0].x = point_exp->value().x();
  marker.points[0].y = point_exp->value().y();
  marker.points[0].z = point_exp->value().z();
  marker.points[1].x = point_exp->value().x() + direction_exp->value().x();
  marker.points[1].y = point_exp->value().y() + direction_exp->value().y();
  marker.points[1].z = point_exp->value().z() + direction_exp->value().z();
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.0;
  marker.color.r = 244.0f / 255.0f;
  marker.color.g = 180.0f / 255.0f;
  marker.color.b = 47.0f / 255.0f;
  marker.color.a = 1.0;

  return marker;
}