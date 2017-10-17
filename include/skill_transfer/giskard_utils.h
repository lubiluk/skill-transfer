#ifndef GISKARD_UTILS
#define GISKARD_UTILS

#include <giskard_core/giskard_core.hpp>
#include "skill_transfer/conversions.h"

inline giskard_core::QPController generateController(const std::string &yaml_string)
{
  // FIXME: add this to giskard_core
  YAML::Node node = YAML::Load(yaml_string);
  giskard_core::QPControllerSpec spec = node.as<giskard_core::QPControllerSpec>();
  giskard_core::QPController controller = giskard_core::generate(spec);

  return controller;
}

inline KDL::Jacobian getJacobian(const giskard_core::QPController &controller,
                                 const std::string &frame_name, const Eigen::VectorXd &observables)
{
  const KDL::Expression<KDL::Frame>::Ptr controlled_frame =
      controller.get_scope().find_frame_expression(frame_name);

  controlled_frame->setInputValues(eigenVectorToStdVector(observables));
  controlled_frame->value();

  KDL::Jacobian jac(15);
  for (size_t i = 0; i < 15; ++i)
    jac.setColumn(i, controlled_frame->derivative(i));

  return jac;
}

#endif
