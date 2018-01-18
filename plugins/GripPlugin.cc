#include "GripPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <string>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GripPlugin);

void GripPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    const auto parentModel = _parent;
    const auto world = parentModel->GetWorld();
    const auto physics = world->GetPhysicsEngine();

    const std::string childLinkName = _sdf->GetElement("childLinkName")->Get<std::string>();
    const std::string parentLinkName = _sdf->GetElement("parentLinkName")->Get<std::string>();
    
    const auto parentLink = parentModel->GetLink(parentLinkName);
    const auto childLink = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(childLinkName));
    
    math::Pose relativePose;
    
    if (_sdf->HasElement("relativePose")) {
      relativePose = _sdf->GetElement("relativePose")->Get<math::Pose>();
      
      const auto parentPose = parentLink->GetWorldPose();
      const auto position = parentPose.CoordPositionAdd(relativePose.pos);
      const auto rotation = parentPose.CoordRotationAdd(relativePose.rot);
      const auto childPose = math::Pose(position, rotation);
      
      childLink->SetWorldPose(childPose);
      
      gzdbg << "Relative pose given " << relativePose << "\n";
      gzdbg << "Parent pose " << parentPose << "\n";
      gzdbg << "Child pose " << childPose << "\n";
    } else {
      relativePose = parentLink->GetWorldPose() - childLink->GetWorldPose();
      
      gzdbg << "Relative pose derrived " << relativePose;
    }

    // Create joint
    const auto joint = physics->CreateJoint("fixed", parentModel);
    // Bullet physics needs accurate joint position
    // ODE does't care
    joint->Load(parentLink, childLink, relativePose);
    joint->Init();
    joint->SetName("grip_joint_" + parentLink->GetScopedName() + "_" + childLink->GetScopedName());
      
    childLink->SetGravityMode(false);
}
