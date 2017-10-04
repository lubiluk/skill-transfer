#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <thread>
#include <mutex>

namespace gazebo
{
class ForceControllerPlugin : public ModelPlugin
{
public:
  ForceControllerPlugin() : ModelPlugin()
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
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
    
    // Link
    this->link_ = _parent->GetLink(this->link_name_);
    
    // Subscribe to the topic
    auto so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      this->topic_name_, 1,
      boost::bind(&ForceControllerPlugin::UpdateObjectVelocity, this, _1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->nh_.subscribe(so);
    
    // Custom Callback Queue
    this->queue_thread_ = std::thread( boost::bind( &ForceControllerPlugin::QueueThread, this ) );
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ForceControllerPlugin::UpdateChild, this, _1));
        
    // Setup a P-controller
    const double P = 10.0;
    const double I = 0;
    const double D = 0;
    
    this->pid_linear_x_ = common::PID(P, I, D);
    this->pid_linear_y_ = common::PID(P, I, D);
    this->pid_linear_z_ = common::PID(P, I, D);
    this->pid_angular_x_ = common::PID(P, I, D);
    this->pid_angular_y_ = common::PID(P, I, D);
    this->pid_angular_z_ = common::PID(P, I, D);
  }
  
  void UpdateObjectVelocity(const geometry_msgs::Twist::ConstPtr& _msg)
  {
    std::lock_guard<std::mutex> lock{this->mutex_};
    
    this->desired_twist_.linear.x = _msg->linear.x;
    this->desired_twist_.linear.y = _msg->linear.y;
    this->desired_twist_.linear.z = _msg->linear.z;
    this->desired_twist_.angular.x = _msg->angular.x;
    this->desired_twist_.angular.y = _msg->angular.y;
    this->desired_twist_.angular.z = _msg->angular.z;
  }
  
  void UpdateChild(const common::UpdateInfo &_info)
  {
    const auto current_sim_time = _info.simTime;
    const auto delta_sim_time = current_sim_time - this->previous_sim_time_;
    
    UpdateObjectForces(delta_sim_time);
  }
  
  void UpdateObjectForces(const common::Time _delta_time)
  {
    std::lock_guard<std::mutex> lock{this->mutex_};
    
    auto current_linear_vel = this->link_->GetWorldLinearVel();
    auto current_angular_vel = this->link_->GetWorldAngularVel();
    
    math::Vector3 force;
    math::Vector3 torque;
    
    force.x = this->pid_linear_x_.Update(current_linear_vel.x - this->desired_twist_.linear.x, _delta_time);
    force.y = this->pid_linear_y_.Update(current_linear_vel.y - this->desired_twist_.linear.y, _delta_time);
    force.z = this->pid_linear_z_.Update(current_linear_vel.z - this->desired_twist_.linear.z, _delta_time);
    
    torque.x = this->pid_angular_x_.Update(current_angular_vel.x - this->desired_twist_.angular.x, _delta_time);
    torque.y = this->pid_angular_y_.Update(current_angular_vel.y - this->desired_twist_.angular.y, _delta_time);
    torque.z = this->pid_angular_z_.Update(current_angular_vel.z - this->desired_twist_.angular.z, _delta_time);
    
    this->link_->SetForce(force);
    this->link_->SetTorque(torque);
  }
  
private:
  std::string link_name_;
  std::string topic_name_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::CallbackQueue queue_;
  std::thread queue_thread_;
  physics::LinkPtr link_;
  std::mutex mutex_;
  geometry_msgs::Twist desired_twist_;
  event::ConnectionPtr update_connection_;
  common::PID pid_linear_x_;
  common::PID pid_linear_y_;
  common::PID pid_linear_z_;
  common::PID pid_angular_x_;
  common::PID pid_angular_y_;
  common::PID pid_angular_z_;
  common::Time previous_sim_time_;
  
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
