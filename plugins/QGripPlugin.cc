#include "QGripPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <string>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(QGripPlugin);

void QGripPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    const auto parentModel = _parent;
    const auto world = parentModel->GetWorld();
    const auto physics = world->GetPhysicsEngine();

    const std::string childLinkName = _sdf->GetElement("childLinkName")->Get<std::string>();
    const std::string parentLinkName = _sdf->GetElement("parentLinkName")->Get<std::string>();
    
    const auto parentLink = parentModel->GetLink(parentLinkName);
    const auto childLink = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(childLinkName));

    gzdbg << "QGrip parent link name: " << parentLink->GetScopedName() << "\n";
    gzdbg << "QGrip child link name: " << childLink->GetScopedName() << "\n";
    
    math::Pose relativePose;
    math::Vector3 relativeTranslation;
    math::Quaternion relativeRotation;
    std::string relativeRotationStr;
    
    if (_sdf->HasElement("relativeTranslation") && _sdf->HasElement("relativeRotationXYZW")) {
      relativeTranslation = _sdf->GetElement("relativeTranslation")->Get<math::Vector3>();
      relativeRotationStr = _sdf->GetElement("relativeRotationXYZW")->Get<std::string>();

      std::istringstream i(relativeRotationStr);
      double x,y,z,w;
      i >> x;
      i >> y;
      i >> z;
      i >> w;

      relativeRotation = math::Quaternion(w, x, y, z);

      gzdbg << "xyzw: " << relativeRotation.x << " " << relativeRotation.y << " " << relativeRotation.z << " " << relativeRotation.w << "\n";
      
      const auto parentPose = parentLink->GetWorldPose();
      const auto rotation = parentPose.CoordRotationAdd(relativeRotation);
      const auto position = parentPose.CoordPositionAdd(relativeTranslation);
      const auto childPose = math::Pose(position, rotation);
      relativePose = math::Pose(relativeTranslation, relativeRotation);
      
      childLink->SetWorldPose(childPose);
      
      gzdbg << "QGrip: Relative pose given, adjusting child pose\n"
            << childPose << "\n";
    } else {
      relativePose = parentLink->GetWorldPose() - childLink->GetWorldPose();
      
      gzdbg << "QGrip: Relative pose derrived\n";
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
