#include "GrainsFactoryPlugin.hh"
#include <gazebo/physics/physics.hh>
#include <sstream>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(GrainsFactoryPlugin)

void GrainsFactoryPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    std::string poseArg = _sdf->GetElement("pose")->GetValue()->GetAsString();
    std::istringstream pss(poseArg);

    double x, y, z, pitch, yaw, roll;
    pss >> x >> y >> z >> roll >> pitch >> yaw;

    math::Pose pose(x, y, z, roll, pitch, yaw);
    
    int quantity = 3;
    double radius = 0.01;
    double mass = 0.001;
    double inertiaDiagonal = 0.4 * mass * radius * radius;
    double friction = 0.4;
    double friction2 = 0.4;
    double velocityDecay = 0.6;
    
    _sdf->GetElement("mass")->GetValue()->Get(mass);
    _sdf->GetElement("radius")->GetValue()->Get(radius);
    _sdf->GetElement("quantity")->GetValue()->Get(quantity);
    _sdf->GetElement("friction")->GetValue()->Get(friction);
    _sdf->GetElement("friction2")->GetValue()->Get(friction2);
    _sdf->GetElement("velocity_decay")->GetValue()->Get(velocityDecay);
    
    for (int i = 0; i < quantity; ++i) {
        std::stringstream xml;
        xml << "<sdf version ='1.6'>\n";
        xml << "<model name ='grain_" << i << "'>\n";
        xml << "\t<pose>" << pose << "</pose>\n";
        xml << "\t\t<link name ='link'>\n";
        xml << "\t\t\t<pose>0 0 0 0 0 0</pose>\n";
        xml << "\t\t\t<inertial>\n";
        xml << "\t\t\t\t<pose>0 0 0 0 0 0</pose>\n";
        xml << "\t\t\t\t<mass>" << mass << "</mass>\n";
        xml << "\t\t\t\t<inertia>\n";
        xml << "\t\t\t\t\t<ixx>" << inertiaDiagonal << "</ixx>";
        xml << "\t\t\t\t\t<ixy>0</ixy>";
        xml << "\t\t\t\t\t<ixz>0</ixz>";
        xml << "\t\t\t\t\t<iyy>" << inertiaDiagonal << "</iyy>";
        xml << "\t\t\t\t\t<iyz>0</iyz>";
        xml << "\t\t\t\t\t<izz>" << inertiaDiagonal << "</izz>";
        xml << "\t\t\t\t</inertia>";
        xml << "\t\t\t</inertial>\n";
        xml << "\t\t\t<velocity_decay>\n";
        xml << "\t\t\t\t<angular>" << velocityDecay << "</angular>\n";
        xml << "\t\t\t</velocity_decay>\n";
        xml << "\t\t\t<collision name ='collision'>\n";
        xml << "\t\t\t\t<geometry>\n";
        xml << "\t\t\t\t\t<sphere>\n";
        xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
        xml << "\t\t\t\t\t</sphere>\n";
        xml << "\t\t\t\t</geometry>\n";
        xml << "\t\t\t\t<surface>\n";
        xml << "\t\t\t\t\t<friction>\n";
        xml << "\t\t\t\t\t\t<ode>\n";
        xml << "\t\t\t\t\t\t\t<mu>" << friction << "</mu>\n";
        xml << "\t\t\t\t\t\t\t<mu2>" << friction2 << "</mu2>\n";
        xml << "\t\t\t\t\t\t</ode>\n";
        xml << "\t\t\t\t\t\t<bullet>\n";
        xml << "\t\t\t\t\t\t\t<friction>" << friction << "</friction>\n";
        xml << "\t\t\t\t\t\t\t<friction2>" << friction2 << "</friction2>\n";
        xml << "\t\t\t\t\t\t</bullet>\n";
        xml << "\t\t\t\t\t</friction>\n";
        xml << "\t\t\t\t</surface>\n";
        xml << "\t\t\t</collision>\n";
        xml << "\t\t\t<visual name ='visual'>\n";
        xml << "\t\t\t\t<geometry>\n";
        xml << "\t\t\t\t\t<sphere>\n";
        xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
        xml << "\t\t\t\t\t</sphere>\n";
        xml << "\t\t\t\t</geometry>\n";
        xml << "\t\t\t\t<material>\n";
        xml << "\t\t\t\t\t<script>\n";
        xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
        xml << "\t\t\t\t\t\t<name>Gazebo/Gold</name>\n";
        xml << "\t\t\t\t\t</script>\n";
        xml << "\t\t\t\t</material>\n";
        xml << "\t\t\t</visual>\n";
        xml << "\t\t</link>\n";
        xml << "</model>\n";
        xml << "</sdf>\n"; 
        
        // Create SDF from the XML string
        sdf::SDF grainSDF;
        grainSDF.SetFromString(xml.str());
        
        // Insert the SDF into the world in runtime
        _parent->InsertModelSDF(grainSDF);
        
        // Translate the position to stack the grains
        pose.pos.z += radius * 2;
    }
}


