#include "StickPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <string>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(StickPlugin);

StickPlugin::StickPlugin(): ModelPlugin(), joint(nullptr) {

}

void StickPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->model = _parent;
    const auto world = this->model->GetWorld();
    this->physics = world->GetPhysicsEngine();
    
    const std::string childLinkName = _sdf->GetElement("childLinkName")->Get<std::string>();
    const std::string parentLinkName = _sdf->GetElement("parentLinkName")->Get<std::string>();
    this->forceThreshold = _sdf->GetElement("force")->Get<double>();

    this->parentLink = this->model->GetLink(parentLinkName);
    this->childLink = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(childLinkName));

    this->CreateJoint();
}

void StickPlugin::OnUpdate(const common::UpdateInfo &_info) {
    if (_info.simTime < 1.0) {
      // Let the stage settle down and position objects
      return;
    }

    auto wrench = this->joint->GetForceTorque(0u);
    auto measuredForce = wrench.body1Force;

    auto force = this->forceThreshold;

    auto measuredForceLength = measuredForce.GetLength();

    if (measuredForceLength > force) {
        gzdbg << "Removed joint: " << " (" << joint->GetName() << "), force: " << measuredForceLength << "\n";
        
        this->BreakJoint();
    }
}

void StickPlugin::Reset() {
    if (this->joint == nullptr) {
        this->CreateJoint();
    }
}

void StickPlugin::CreateJoint() {
    this->joint = this->physics->CreateJoint("fixed", this->model);
    // Bullet physics needs accurate joint position
    // ODE does't care
    this->joint->Load(this->parentLink, this->childLink, this->parentLink->GetWorldPose() - this->childLink->GetWorldPose());
    this->joint->Init();
    this->joint->SetProvideFeedback(true);
    this->joint->SetName("stick_joint_" + this->parentLink->GetScopedName() + "_" + this->childLink->GetScopedName());
    
    // Disable gravity on the butter link
    this->parentLink->SetGravityMode(false);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&StickPlugin::OnUpdate, this, _1));
}

void StickPlugin::BreakJoint() {
    this->joint->Detach();
    this->joint = nullptr;
    
    // Enable gravity on the childLink
    this->parentLink->SetGravityMode(true);

    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    this->updateConnection = nullptr;
}

