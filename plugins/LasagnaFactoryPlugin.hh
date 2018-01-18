#ifndef PLUGINS_LASAGNAFACTORYPLUGIN_H
#define PLUGINS_LASAGNAFACTORYPLUGIN_H


#include <gazebo/gazebo.hh>

namespace gazebo {
    class LasagnaFactoryPlugin : public WorldPlugin {
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override;
    };
}


#endif //PLUGINS_LASAGNAFACTORYPLUGIN_H
