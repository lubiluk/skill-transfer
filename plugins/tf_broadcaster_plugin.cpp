#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <thread>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{
class TfBroadcasterPlugin : public ModelPlugin
{
public:
  TfBroadcasterPlugin() : ModelPlugin()
  {
  }
  
  ~TfBroadcasterPlugin() 
  {
    
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // SDF values
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

    // Link
    link_ = _parent->GetLink(this->link_name_);
    
    // Custom Callback Queue
    queue_thread_ = std::thread( boost::bind( &TfBroadcasterPlugin::QueueThread, this ) );
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&TfBroadcasterPlugin::UpdateChild, this, _1));
  }
  
  void UpdateChild(const common::UpdateInfo &_info)
  {
    const auto current_sim_time = _info.simTime;
    const auto delta_sim_time = current_sim_time - this->previous_sim_time_;
    
    PublishTf(delta_sim_time);
  }
  
  void PublishTf(const common::Time _delta_time)
  {
    math::Pose pose = link_->GetWorldPose();
    
    geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = frame_name_;
    transformStamped.transform.translation.x = pose.pos.x;
    transformStamped.transform.translation.y = pose.pos.y;
    transformStamped.transform.translation.z = pose.pos.z;
   
    transformStamped.transform.rotation.x = pose.rot.x;
    transformStamped.transform.rotation.y = pose.rot.y;
    transformStamped.transform.rotation.z = pose.rot.z;
    transformStamped.transform.rotation.w = pose.rot.w;

    br_.sendTransform(transformStamped);
  }
  
private:
  std::string link_name_;
  std::string frame_name_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  std::thread queue_thread_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  common::Time previous_sim_time_;
  tf2_ros::TransformBroadcaster br_;
  
  void QueueThread()
  {
    static const double timeout = 0.01;

    while (this->nh_.ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(TfBroadcasterPlugin)
}
