// Source: http://answers.gazebosim.org/question/3383/how-to-add-a-dynamic-visual-marker-in-gazebo/

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>


namespace gazebo
{
namespace rendering
{
class GiskardVisualizationPlugin : public VisualPlugin
{
private:
  /// \brief pointer to ros node
  ros::NodeHandle* rosnode_;

  /// \brief store model name
  std::string model_name_;

  /// \brief topic name
  std::string topic_name_;

  // /// \brief The visual pointer used to visualize the force.
  VisualPtr visual_;

  // /// \brief The scene pointer.
  ScenePtr scene_;

  /// \brief For example a line to visualize the force
  DynamicLines *line;

  /// \brief for setting ROS name space
  std::string visual_namespace_;

  /// \Subscribe to some force
  ros::Subscriber force_sub_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;
  
public:
  /// \brief Constructor
  GiskardVisualizationPlugin(): line(nullptr)
  {
  
  }

  /// \brief Destructor
  virtual ~GiskardVisualizationPlugin() 
  {
    // Finalize the visualizer
    this->rosnode_->shutdown();
    delete this->rosnode_;
  }

  /// \brief Load the visual force plugin tags
  /// \param node XML config node
  void Load( VisualPtr _parent, sdf::ElementPtr _sdf )
  {
    this->visual_ = _parent;
    this->visual_namespace_ = "visual/";

    // start ros node
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    }

    this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
    this->force_sub_ = this->rosnode_->subscribe("/some_force", 1000, &GiskardVisualizationPlugin::VisualizeForceOnLink, this);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectRender(
        boost::bind(&GiskardVisualizationPlugin::UpdateChild, this));
  }


protected: 
  /// \brief Update the visual plugin
  virtual void UpdateChild()
  {
    ros::spinOnce();
  }
  
private:
  /// \brief Visualize the force
  void VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
  {
//    this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

//    //TODO: Get the current link position
//    link_pose = CurrentLinkPose();
//    //TODO: Get the current end position
//    endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

//    // Add two points to a connecting line strip from link_pose to endpoint
//    this->line->AddPoint(
//      math::Vector3(
//        link_pose.position.x,
//        link_pose.position.y,
//        link_pose.position.z
//        )
//      );
//    this->line->AddPoint(math::Vector3(endpoint.x, endpoint.y, endpoint.z));
//    // set the Material of the line, in this case to purple
//    this->line->setMaterial("Gazebo/Purple")
//    this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
//    this->visual_->SetVisible(true);
  }
  
};

// Register this plugin within the simulator
GZ_REGISTER_VISUAL_PLUGIN(GiskardVisualizationPlugin)
    
}
}
