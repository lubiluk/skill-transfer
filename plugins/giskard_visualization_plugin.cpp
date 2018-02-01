#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <boost/format.hpp>
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <visualization_msgs/Marker.h>

namespace gazebo
{
class GiskardVisualizationPlugin : public WorldPlugin
{
private:
  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> node_handle_;
  /// \brief A ROS subscriber
  ros::Subscriber subscriber_;
  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue callback_queue_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  std::map<std::string, common::Time> markers;

public:
  GiskardVisualizationPlugin() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->world_ = _world;

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->node_handle_.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<visualization_msgs::Marker>(
            "/giskard/visualization_marker",
            10,
            boost::bind(&GiskardVisualizationPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->callback_queue_);
    this->subscriber_ = this->node_handle_->subscribe(so);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GiskardVisualizationPlugin::Update, this));
  }

  void Update()
  {
    // Get new commands/state
    callback_queue_.callAvailable();
  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
  void OnRosMsg(const visualization_msgs::MarkerConstPtr &_msg)
  {
    if (_msg->type != visualization_msgs::Marker::SPHERE)
    {
      return;
    }

    std::string name = _msg->ns;

    if (markers.find(name) == markers.end())
    {
      std::string pose = boost::str(boost::format("%1% %2% %3% 0 0 0") %
                                    (_msg->pose.position.x) %
                                    (_msg->pose.position.y) %
                                    (_msg->pose.position.z));
      sdf::SDF sphereSDF;
      sphereSDF.SetFromString(
          "<sdf version ='1.6'>\
          <model name ='sphere'>\
            <static>true</static>\
            <pose>" +
          pose + "</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.005</radius></sphere>\
                </geometry>\
                <material>\
                    <script>\
                        <name>Gazebo/Yellow</name>\
                        <uri>file://media/materials/scripts/gazebo.material</uri>\
                    </script>\
                </material>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
      // Demonstrate using a custom model name.
      sdf::ElementPtr modelSDF = sphereSDF.Root()->GetElement("model");
      modelSDF->GetAttribute("name")->SetFromString(name);
      this->world_->InsertModelSDF(sphereSDF);
    }
    else
    {
      auto model = this->world_->GetModel(name);

      if (model)
      {
        math::Pose pose(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z, 0.0, 0.0, 0.0);
        model->SetWorldPose(pose);
      }
    }

    markers[name] = common::Time::GetWallTime();
  }
};

GZ_REGISTER_WORLD_PLUGIN(GiskardVisualizationPlugin)
}