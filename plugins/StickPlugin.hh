#ifndef PLUGINS_STICKPLUGIN_H
#define PLUGINS_STICKPLUGIN_H


#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>

namespace gazebo {
    class StickPlugin : public ModelPlugin {
    public:
        StickPlugin();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
        void OnUpdate(const common::UpdateInfo & _info);
        void Reset() override;
        void CreateJoint();
        void BreakJoint();

    private:
        physics::PhysicsEnginePtr physics;
        physics::ModelPtr model;
        physics::JointPtr joint;
        physics::LinkPtr childLink;
        physics::LinkPtr parentLink;
        event::ConnectionPtr updateConnection;
        double forceThreshold;
    };
}


#endif //PLUGINS_STICKPLUGIN_H
