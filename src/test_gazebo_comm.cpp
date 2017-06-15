#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <giskard_core/giskard_core.hpp>
#include <giskard_ros_utils/giskard_ros_utils.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <visualization_msgs/Marker.h>

template<class T, class U>
inline std::map<T, U> to_map(const std::vector<T>& keys, const std::vector<U>& values)
{
  // FIXME: move move to another package
  if (keys.size() != values.size())
    throw std::runtime_error("Number of keys not equal to numbers of values.");

  std::map<T, U> result;
  for (size_t i=0; i<keys.size(); ++i)
    result.insert(std::pair<T,U>(keys[i], values[i]));

  return result;
}

giskard_core::QPController generate_controller(const std::string& yaml_string)
{
  // FIXME: add this to giskard_core
  YAML::Node node = YAML::Load(yaml_string);
  giskard_core::QPControllerSpec spec = node.as<giskard_core::QPControllerSpec>();
  giskard_core::QPController controller = giskard_core::generate(spec);

  return controller;
}

inline Eigen::VectorXd pose_to_giskard(const geometry_msgs::Pose& pose) 
{
  // FIXME: refactor this into a header file
  KDL::Frame frame;
  tf::poseMsgToKDL(pose, frame);

  Eigen::VectorXd result(6);

  result(0) = pose.position.x;
  result(1) = pose.position.y;
  result(2) = pose.position.z;

  KDL::Rotation::Quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w).GetEulerZYX(result(3), result(4), result(5));

  return result;
}

inline Eigen::VectorXd frame_to_giskard(const KDL::Frame& frame)
{
  Eigen::VectorXd result(6);

  result(0) = frame.p.x();
  result(1) = frame.p.y();
  result(2) = frame.p.z();
  frame.M.GetEulerZYX(result(3), result(4), result(5));

  return result;
}

inline std::vector<double> to_stl(const Eigen::VectorXd& v)
{
  // FIXME: where to put this?
  std::vector<double> result;
  for(size_t i=0; i<v.rows(); ++i)
    result.push_back(v(i));

  return result;
}

inline KDL::Jacobian get_jacobian(const giskard_core::QPController& controller,
    const std::string& frame_name, const Eigen::VectorXd& observables)
{
  const KDL::Expression<KDL::Frame>::Ptr controlled_frame_ =
    controller.get_scope().find_frame_expression(frame_name);

  controlled_frame_->setInputValues(to_stl(observables));
  controlled_frame_->value();
        
  KDL::Jacobian jac(6);
  for(size_t i=0; i<6; ++i)
    jac.setColumn(i, controlled_frame_->derivative(i));
          
  return jac;
}

inline geometry_msgs::Twist giskard_to_msg(const Eigen::VectorXd& t)
{
  // FIXME: where to put this?
  if (t.rows() != 6)
    throw std::runtime_error("Did not receive vector representing a twist with 6 values.");

  geometry_msgs::Twist result;
  result.linear.x = t(0);
  result.linear.y = t(1);
  result.linear.z = t(2);
  result.angular.x = t(3);
  result.angular.y = t(4);
  result.angular.z = t(5);

  return result;
}

inline visualization_msgs::Marker vector_to_point_marker(const KDL::Vector &v, std::string ns, int id)
{
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = v.x();
  marker.pose.position.y = v.y();
  marker.pose.position.z = v.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  return marker;
}

class TestGazeboComm
{
  public:
    TestGazeboComm(const ros::NodeHandle& nh): 
      nh_(nh), 
      controller_(generate_controller(giskard_ros_utils::readParam<std::string>(nh_, "yaml_string"))),
      sub_(nh_.subscribe("/gazebo/link_states", 1, &TestGazeboComm::callback, this)),
      pub_(nh_.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1)),
      pub_viz_(nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1)),
      controller_started_(false)
    {
    }

    ~TestGazeboComm() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub_viz_;
    giskard_core::QPController controller_;
    bool controller_started_;

    void callback(const gazebo_msgs::LinkStatesConstPtr& msg)
    {
      std::map<std::string, geometry_msgs::Pose> link_state =
        to_map<std::string, geometry_msgs::Pose>(msg->name, msg->pose);

      auto knife_world_pose = link_state.find("knife::link")->second;
      auto gripper_world_pose = link_state.find("gripper::link")->second;

      KDL::Frame knife_world_frame;
      tf::poseMsgToKDL(knife_world_pose, knife_world_frame);

      KDL::Frame gripper_world_frame;
      tf::poseMsgToKDL(gripper_world_pose, gripper_world_frame);
      
      auto knife_gripper_frame = gripper_world_frame.Inverse() * knife_world_frame;

      Eigen::VectorXd inputs(18);
      // FIXME: protect against not finding
      inputs.segment(0, 6) = pose_to_giskard(gripper_world_pose);
      inputs.segment(6, 6) = pose_to_giskard(link_state.find("frying_pan::link")->second);
      inputs.segment(12, 6) = frame_to_giskard(knife_gripper_frame);

      if (!controller_started_)
      {
        // FIXME: get nWSR from parameter server
        if (!controller_.start(inputs, 100))
          throw std::runtime_error("Failed to start controller.");
        controller_started_ = true;
      }

      // FIXME: get nWSR from parameter server
      if (!controller_.update(inputs, 100))
        throw std::runtime_error("Failed to update controller.");
      
      gazebo_msgs::LinkState cmd;
      cmd.link_name = "gripper::link";
      cmd.reference_frame = "world";
      for (size_t i=0; i<msg->name.size(); ++i)
        if (msg->name[i].compare(cmd.link_name) == 0)
          cmd.pose = msg->pose[i];

      cmd.twist = giskard_to_msg(get_jacobian(controller_, "gripper-frame", inputs).data * controller_.get_command());
      pub_.publish(cmd);

      // Visualization
      const KDL::Expression<KDL::Vector>::Ptr frying_pan_edge_expression =
        controller_.get_scope().find_vector_expression("frying-pan-edge");
      KDL::Vector frying_pan_edge = frying_pan_edge_expression->value();

      visualization_msgs::Marker marker = vector_to_point_marker(frying_pan_edge, "frying_pan_edge", 1);

      pub_viz_.publish(marker);

      const KDL::Expression<KDL::Vector>::Ptr knife_base_expression =
        controller_.get_scope().find_vector_expression("knife-base");
      KDL::Vector knife_base = knife_base_expression->value();

      visualization_msgs::Marker marker2 = vector_to_point_marker(knife_base, "knife_base", 1);

      pub_viz_.publish(marker2);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_test_comm");
    ros::NodeHandle nh("~");

    try
    {
      TestGazeboComm tgc(nh);
      ros::spin();
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
                        
    return 0;
}
