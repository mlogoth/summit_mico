

#ifndef PLATFORM_STATE_CONTROLLER_HPP
#define PLATFORM_STATE_CONTROLLER_HPP

#include <cstddef>
#include <vector>
#include <string>
#include <fstream>

// Boost LibrariesW
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/String.h>

#include "tf/transform_datatypes.h"

#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>


namespace summit_mico_control
{

class platform_state_controller : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{

public:
    platform_state_controller(void);
    ~platform_state_controller(void);

    
    void read_xml_robot_params(const ros::NodeHandle &_node);
    void read_xml_controller_params(const ros::NodeHandle &_node);
    
    // Initialization Function
    bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
    // Starting Routine
    void starting(const ros::Time &time);
    // Update Routine
    void update(const ros::Time &time, const ros::Duration &duration);

    void stopping(const ros::Time &time);

    void eulerToQuatMsg(const double &roll, const double &pitch, const double &yaw, geometry_msgs::Quaternion &msg_quat);
    void init_ros_communication(const ros::NodeHandle& n);
    void getCommand(const geometry_msgs::Twist::ConstPtr &msg);
    // Function that converts strings to lowercase.
    std::string str_tolower(std::string s);
    double constrainAngle(double x);
    std::string getLeafNamespace(const ros::NodeHandle& nh);
    std::string getRootNamespace(const ros::NodeHandle& nh);




private:
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::VelocityJointInterface *robot_;

    // Joint names
    std::vector<std::string> joint_names_;

    // Base Pose and Velocity
    std::vector<double> base_pose_, base_velocity_;
    geometry_msgs::Quaternion base_orientation_;
    
    // Type of platform modes
    std::string platform_mode_ = "omni"; 
    std::list<std::string> kinematic_modes_ = {"omni", "diff", "skid", "force_diff"};

    // Velocity Command Topic 
    std::string command_topic_="cmd_vel"; 
    std::string odom_topic_="odom";

    // Frame Base and Odometry
    std::string odom_frame_="odom";
    std::string base_frame_= "base_footprint";

    // Boolean Publish Odometry
    bool publish_odom_ = false;
    bool broadcast_tf_ = false;

    // Command Velocity
    std::vector<double> command_velocity_;

    // State Variables 
    std::vector<double> qpos, qvel, qeff;
    std::vector<std::string> qnam;

    // ROS Variables
    ros::NodeHandle node_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_sub_;
    double begin;

};

} 
#endif