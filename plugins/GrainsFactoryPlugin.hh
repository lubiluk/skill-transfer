#ifndef PLUGINS_GRAINSFACTORYPLUGIN_H
#define PLUGINS_GRAINSFACTORYPLUGIN_H


#include <gazebo/gazebo.hh>

namespace gazebo {
    class GrainsFactoryPlugin : public WorldPlugin {
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override;
    };
}


#endif //PLUGINS_GRAINSFACTORYPLUGIN_H
