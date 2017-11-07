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

namespace gazebo
{
class ForceControllerPlugin : public ModelPlugin
{
public:
  ForceControllerPlugin() : ModelPlugin(), P_(0.0), I_(0.0), D_(0.0), tfListener(tfBuffer)
  {
  }
  
  ~ForceControllerPlugin() 
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
    this->link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    this->target_frame_name_ = _sdf->GetElement("targetFrameName")->Get<std::string>();
    this->base_frame_name_ = _sdf->GetElement("baseFrameName")->Get<std::string>();
    this->P_ = _sdf->GetElement("P")->Get<double>();
    this->I_ = _sdf->GetElement("I")->Get<double>();
    this->D_ = _sdf->GetElement("D")->Get<double>();
    
    // Link
    this->link_ = _parent->GetLink(this->link_name_);
    
    // Custom Callback Queue
    this->queue_thread_ = std::thread( boost::bind( &ForceControllerPlugin::QueueThread, this ) );
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ForceControllerPlugin::UpdateChild, this, _1));
    
    this->pid_linear_x_ = common::PID(P_, I_, D_, 1000, -1000);
    this->pid_linear_y_ = common::PID(P_, I_, D_, 1000, -1000);
    this->pid_linear_z_ = common::PID(P_, I_, D_, 1000, -1000);
    this->pid_angular_x_ = common::PID(P_, I_, D_, 1000, -1000);
    this->pid_angular_y_ = common::PID(P_, I_, D_, 1000, -1000);
    this->pid_angular_z_ = common::PID(P_, I_, D_, 1000, -1000);
  }
  
  void UpdateChild(const common::UpdateInfo &_info)
  {
    const auto current_sim_time = _info.simTime;
    const auto delta_sim_time = current_sim_time - this->previous_sim_time_;
    
    UpdateObjectForces(delta_sim_time);
  }
  
  void UpdateObjectForces(const common::Time _delta_time)
  {
    geometry_msgs::TransformStamped transformStamped;
    
    try 
    {
      transformStamped = tfBuffer.lookupTransform(
        this->base_frame_name_, this->target_frame_name_, ros::Time(0));  
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      return;
    }
    
    const auto current_pose = this->link_->GetWorldPose();
    const math::Pose desired_pose = math::Pose(
      math::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z), 
      math::Quaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z));
    math::Vector3 force;
    math::Vector3 torque;
    
//    force.x = this->pid_linear_x_.Update(current_pose.pos.x - desired_pose.pos.x, _delta_time);
//    force.y = this->pid_linear_y_.Update(current_pose.pos.y - desired_pose.pos.y, _delta_time);
//    force.z = this->pid_linear_z_.Update(current_pose.pos.z - desired_pose.pos.z, _delta_time);
    
//    ROS_INFO_STREAM("Current pos: " << current_pose.pos.x << " " << current_pose.pos.y << " " << current_pose.pos.z);
//    ROS_INFO_STREAM("Desired pos: " << desired_pose.pos.x << " " << desired_pose.pos.y << " " << desired_pose.pos.z);
//    ROS_INFO_STREAM("Error: " << current_pose.pos.y - desired_pose.pos.y);
////    
//    torque.x = this->pid_angular_x_.Update(current_pose.rot.x - transformStamped.transform.rotation.x, _delta_time);
//    torque.y = this->pid_angular_y_.Update(current_pose.rot.y - transformStamped.transform.rotation.y, _delta_time);
//    torque.z = this->pid_angular_z_.Update(current_pose.rot.z - transformStamped.transform.rotation.z, _delta_time);
//    
//    this->link_->SetForce(force);
//    this->link_->SetTorque(torque);

    this->link_->SetWorldPose(desired_pose);
  }
  
private:
  std::string link_name_;
  std::string target_frame_name_;
  std::string base_frame_name_;
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  std::thread queue_thread_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  common::PID pid_linear_x_;
  common::PID pid_linear_y_;
  common::PID pid_linear_z_;
  common::PID pid_angular_x_;
  common::PID pid_angular_y_;
  common::PID pid_angular_z_;
  common::Time previous_sim_time_;
  // Setup a P-controller
  double P_;
  double I_;
  double D_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  
  void QueueThread()
  {
    static const double timeout = 0.01;

    while (this->nh_.ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(ForceControllerPlugin)
}
