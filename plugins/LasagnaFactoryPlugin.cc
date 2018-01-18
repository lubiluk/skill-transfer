/**
 * Lasagna factory
 *
 * Credit:
 * Based on Paulo Abelha's lasagna factory.
 * https://github.com/pauloabelha/gazebo_tasks/blob/master/cutting_lasagna/plugins/factory_lasagna.cc
 */
#include "LasagnaFactoryPlugin.hh"
#include <gazebo/physics/physics.hh>
#include <sstream>
#include <random>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(LasagnaFactoryPlugin)

void LasagnaFactoryPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    math::Pose pose {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    math::Vector3 size {5.0, 5.0, 5.0};
    double radius {0.01};
    double mass {0.5};
    double friction {0.4};
    double friction2 {0.4};
    double cfm {0.0};
    double erp {0.0};
    double jointDamping {0.0};
    double jointFriction {0.0};
    double spotProbability {0.4};
    
    // Read values from XML if available
    if (_sdf->HasElement("pose"))
      pose = _sdf->GetElement("pose")->Get<math::Pose>();
      
    if (_sdf->HasElement("size"))
      size = _sdf->GetElement("size")->Get<math::Vector3>();
      
    if (_sdf->HasElement("radius"))
      radius = _sdf->GetElement("radius")->Get<double>();
      
    if (_sdf->HasElement("mass"))
      mass = _sdf->GetElement("mass")->Get<double>();
      
    if (_sdf->HasElement("friction"))
      friction = _sdf->GetElement("friction")->Get<double>();
      
    if (_sdf->HasElement("friction2"))
      friction2 = _sdf->GetElement("friction2")->Get<double>();
      
    if (_sdf->HasElement("cfm"))
      cfm = _sdf->GetElement("cfm")->Get<double>();
      
    if (_sdf->HasElement("erp"))
      erp = _sdf->GetElement("erp")->Get<double>();
      
    if (_sdf->HasElement("jointDamping"))
      jointDamping = _sdf->GetElement("jointDamping")->Get<double>();
      
    if (_sdf->HasElement("jointFriction"))
      jointFriction = _sdf->GetElement("jointFriction")->Get<double>();
      
    if (_sdf->HasElement("spotProbability"))
      spotProbability = _sdf->GetElement("spotProbability")->Get<double>();
    
    //
    double xShift = -(size.x - 1) / 2 * radius;
    double yShift = -(size.y - 1) / 2 * radius;
    double zShift = -(size.z - 1) / 2 * radius;
//    double diameter = 2 * radius;
    double sphereMass = mass / (size.x * size.y * size.z);
//    double inertiaDiagonal = 0.4 * sphereMass * radius * radius;

    std::stringstream xml;
    xml << "<sdf version ='1.6'>\n";
    xml << "<model name ='lasagna'>\n";
    xml << "\t<pose>" << pose << "</pose>\n";
    
    for (int i = 0; i < size.x; ++i) {
        for (int j = 0; j < size.y; ++j) {
            for (int k = 0; k < size.z; ++k) {
                std::string index = std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);

                std::string color = "Yellow";
                
                if (rand() % 100 + 1 <= (spotProbability * 100))
                    color = "Red";

                xml << "\t\t<link name ='link_" << index << "'>\n";
                xml << "\t\t\t<pose>"
                                << radius * i + xShift << " "
                                << radius * j + yShift << " "
                                << radius * k + zShift << " 0 0 0"
                            "</pose>\n";
                xml << "\t\t\t<inertial>\n";
                xml << "\t\t\t\t<pose>0 0 0 0 0 0</pose>\n";
                xml << "\t\t\t\t<mass>" << sphereMass << "</mass>\n";
                // The default inertia keeps the lasagna stable
//                xml << "\t\t\t\t<inertia>\n";
//                xml << "\t\t\t\t\t<ixx>" << inertiaDiagonal << "</ixx>";
//                xml << "\t\t\t\t\t<ixy>0</ixy>";
//                xml << "\t\t\t\t\t<ixz>0</ixz>";
//                xml << "\t\t\t\t\t<iyy>" << inertiaDiagonal << "</iyy>";
//                xml << "\t\t\t\t\t<iyz>0</iyz>";
//                xml << "\t\t\t\t\t<izz>" << inertiaDiagonal << "</izz>";
//                xml << "\t\t\t\t</inertia>";
                xml << "\t\t\t</inertial>\n";
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
                xml << "\t\t\t\t\t<contact>\n";
                xml << "\t\t\t\t\t\t<ode>\n";
                xml << "\t\t\t\t\t\t\t<soft_cfm>" << cfm << "</soft_cfm>\n";
                xml << "\t\t\t\t\t\t\t<soft_erp>" << erp << "</soft_erp>\n";
                xml << "\t\t\t\t\t\t</ode>\n";
                xml << "\t\t\t\t\t\t<bullet>\n";
                xml << "\t\t\t\t\t\t\t<soft_cfm>" << cfm << "</soft_cfm>\n";
                xml << "\t\t\t\t\t\t\t<soft_erp>" << erp << "</soft_erp>\n";
                xml << "\t\t\t\t\t\t</bullet>\n";
                xml << "\t\t\t\t\t</contact>\n";
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
                xml << "\t\t\t\t\t\t<name>Gazebo/" << color << "</name>\n";
                xml << "\t\t\t\t\t</script>\n";
                xml << "\t\t\t\t</material>\n";
                xml << "\t\t\t</visual>\n";
                xml << "\t\t</link>\n";
            }
        }
    }
    
    for (int i = 0; i < size.x; ++i) {
        for (int j = 0; j < size.y; ++j) {
            for (int k = 1; k < size.z; ++k) {
                const auto currentIndex = std::to_string(i) + "_" 
                                        + std::to_string(j) + "_" 
                                        + std::to_string(k);
                const auto previousIndex = std::to_string(i) + "_" 
                                         + std::to_string(j) + "_" 
                                         + std::to_string(k - 1);
                
                xml << "\t\t<joint name ='joint_" << currentIndex << "_" << previousIndex << "' type='prismatic'>\n";
                xml << "\t\t\t\t<pose>0 0 0.03 0 0 0</pose>\n";
                xml << "\t\t\t\t<parent>link_" << previousIndex << "</parent>\n";
                xml << "\t\t\t\t<child>link_" << currentIndex << "</child>\n";
                xml << "\t\t\t\t<axis>\n";
                xml << "\t\t\t\t\t<dynamics>\n";
                xml << "\t\t\t\t\t\t<damping>" << jointDamping << "</damping>\n";
                xml << "\t\t\t\t\t\t<friction>" << jointFriction << "</friction>\n";
                xml << "\t\t\t\t\t</dynamics>\n";
                xml << "\t\t\t\t\t<xyz>0 0 1</xyz>\n";
                xml << "\t\t\t\t</axis>\n";
                xml << "\t\t</joint>\n";
            }
        }
    }
    
    xml << "</model>\n";
    xml << "</sdf>\n"; 
    
    // Create SDF from the XML string
    sdf::SDF model;
    model.SetFromString(xml.str());
    
    // Insert the SDF into the world in runtime
    _parent->InsertModelSDF(model);
}


