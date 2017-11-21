#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
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
    
    const auto gains = _sdf->GetElement("gains");
    const auto linearGains = gains->GetElement("linear");
    this->linear_P_ = linearGains->GetElement("P")->Get<double>();
    this->linear_I_ = linearGains->GetElement("I")->Get<double>();
    this->linear_D_ = linearGains->GetElement("D")->Get<double>();
    const auto angularGains = gains->GetElement("angular");
    this->angular_P_ = angularGains->GetElement("P")->Get<double>();
    this->angular_I_ = angularGains->GetElement("I")->Get<double>();
    this->angular_D_ = angularGains->GetElement("D")->Get<double>();
    
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
    
    this->pid_linear_x_ = common::PID(linear_P_, linear_I_, linear_D_);
    this->pid_linear_y_ = common::PID(linear_P_, linear_I_, linear_D_);
    this->pid_linear_z_ = common::PID(linear_P_, linear_I_, linear_D_);
    this->pid_angular_x_ = common::PID(angular_P_, angular_I_, angular_D_);
    this->pid_angular_y_ = common::PID(angular_P_, angular_I_, angular_D_);
    this->pid_angular_z_ = common::PID(angular_P_, angular_I_, angular_D_);
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
    
    if (current_sim_time < 1) return;
    
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
  // Setup a P-controller
  double linear_P_;
  double linear_I_;
  double linear_D_;
  double angular_P_;
  double angular_I_;
  double angular_D_;
  
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
