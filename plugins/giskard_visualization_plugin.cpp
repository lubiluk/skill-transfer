#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <visualization_msgs/Marker.h>
#include <boost/format.hpp>
#include <map>
#include <string>
#include <mutex>
#include <thread>

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
  ros::CallbackQueue queue_;
  std::thread queue_thread_;
  std::mutex mutex_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  std::map<std::string, visualization_msgs::Marker> markers_;

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
            ros::VoidPtr(), &this->queue_);
    this->subscriber_ = this->node_handle_->subscribe(so);
    
    // Custom Callback Queue
    this->queue_thread_ = std::thread( boost::bind( &GiskardVisualizationPlugin::QueueThread, this ) );

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GiskardVisualizationPlugin::Update, this));
  }

  void Update()
  {
    std::lock_guard<std::mutex> lock{this->mutex_};

    for (const auto p : this->markers_)
    {
      const visualization_msgs::Marker &msg = p.second;
      const std::string &name = msg.ns;
      auto model = this->world_->GetModel(name);

      if (model)
      {
        updateMarkerModel(model, msg);
      }
      else
      {
        createMarkerModel(msg);
      }
    }
  }

  void createMarkerModel(const visualization_msgs::Marker &_msg)
  {
    const std::string &name = _msg.ns;

    std::string pose = boost::str(boost::format("%1% %2% %3% 0 0 0") %
                                  (_msg.pose.position.x) %
                                  (_msg.pose.position.y) %
                                  (_msg.pose.position.z));
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
        "<sdf version ='1.6'>\
          <model name ='sphere'>\
            <static>true</static>\
            <pose>" + pose + "</pose>\
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

    sdf::ElementPtr modelSDF = sphereSDF.Root()->GetElement("model");
    modelSDF->GetAttribute("name")->SetFromString(name);
    this->world_->InsertModelSDF(sphereSDF);

    gzdbg << "Created Marker: " << name << "\n";
  }

  void updateMarkerModel(physics::ModelPtr model, const visualization_msgs::Marker &_msg)
  {
      math::Pose pose(_msg.pose.position.x,
                      _msg.pose.position.y,
                      _msg.pose.position.z,
                      0.0, 0.0, 0.0);
      model->SetWorldPose(pose);
  }

  /// \brief Handle an incoming message from ROS
  void OnRosMsg(const visualization_msgs::MarkerConstPtr &_msg)
  {
    if (_msg->type != visualization_msgs::Marker::SPHERE)
    {
      return;
    }

    std::lock_guard<std::mutex> lock{this->mutex_};

    this->markers_[_msg->ns] = *_msg;
  }

private:
  void QueueThread()
  {
    static const double timeout = 0.01;

    while (this->node_handle_->ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
};

GZ_REGISTER_WORLD_PLUGIN(GiskardVisualizationPlugin)
}