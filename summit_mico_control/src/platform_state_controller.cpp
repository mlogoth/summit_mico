#include <summit_mico_control/platform_state_controller.hpp>

namespace summit_mico_control
{

platform_state_controller::platform_state_controller(void)
{
    ROS_ERROR("platform_state_controller initialization");
    node_ = ros::NodeHandle();
}

platform_state_controller::~platform_state_controller(void){};

/*
* Read From Parameter Server Robot Parameters
* joint names, joint limits, robot description    
* */

void platform_state_controller::read_xml_robot_params(const ros::NodeHandle &_node)
{
    /*
    * Get Joint Names for the controller
    */
    XmlRpc::XmlRpcValue joint_names;
    if (!node_.getParam("platform_state_controller/joints", joint_names))
    {
        ROS_ERROR("platform_state_controller: No 'joints' defined in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
    }

    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("platform_state_controller: 'joints' is not a struct. (namespace: %s)",
                  node_.getNamespace().c_str());
    }

    std::cout << "Joint Names Parameter Structure Size: " << joint_names.size() << std::endl;

    for (int i = 0; i < joint_names.size(); i++)
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("platform_state_controller: joints are not strings. (namespace: %s)",
                      node_.getNamespace().c_str());
            //return false;
        }
        joint_names_.push_back((std::string)name_value);
        std::cout << joint_names_[i] << std::endl;
    }

    /*
    * Get Platform Type
    */
    XmlRpc::XmlRpcValue platform_mode;
    if (!node_.getParam("platform_state_controller/mode", platform_mode))
    {
        ROS_ERROR("platform_state_controller: No 'mode' defined in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
    }
    if (platform_mode.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        ROS_ERROR("platform_state_controller: 'mode' is not a string. (namespace: %s)",
                  node_.getNamespace().c_str());
    }
    else
    {
        // Convert to lowercase and check if it is valid
        platform_mode_ = platform_state_controller::str_tolower(platform_mode);
        if (not(std::find(kinematic_modes_.begin(), kinematic_modes_.end(), platform_mode_) != kinematic_modes_.end()))
        {
            ROS_ERROR("platform_state_controller: Wrong Kinematic Mode Defined");
            platform_mode_ = "omni";
        }
    }

    // Get Odom Frame
    if (node_.hasParam("platform_state_controller/odom_frame"))
    {
        XmlRpc::XmlRpcValue odom_frame;
        node_.getParam("platform_state_controller/odom_frame", odom_frame);
        if (odom_frame.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("platform_state_controller: 'odom_frame' is not a string. (namespace: %s)",
                      node_.getNamespace().c_str());
        }
        odom_frame_ = (std::string)odom_frame;
        ROS_INFO("platform_state_controller: Odometry Frame Name %s", odom_frame_);
    }
    else
    {
        ROS_WARN("Parameter 'Odometry Frame' not set, set the default one: 'odom'");
        odom_frame_ = "odom";
    }

    // Get Base Robot Frame
    if (node_.hasParam("platform_state_controller/base_frame"))
    {
        XmlRpc::XmlRpcValue base_frame;
        node_.getParam("platform_state_controller/base_frame", base_frame);
        if (base_frame.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("platform_state_controller: 'base_frame' is not a string. (namespace: %s)",
                      node_.getNamespace().c_str());
        }
        base_frame_ = (std::string)base_frame;
        ROS_INFO("platform_state_controller: Base Frame Name %s", base_frame_);
    }
    else
    {
        ROS_WARN("Parameter 'Base Frame' not set, set the default one: 'base_footprint'");
        base_frame_ = "base_footprint";
    }
}

/*
* Read From Parameter Server Controller Parameters
* Gains, Type, Space    
* */
void platform_state_controller::read_xml_controller_params(const ros::NodeHandle &_node)
{
    // Get Command Topic
    if (node_.hasParam("platform_state_controller/command_topic"))
    {
        XmlRpc::XmlRpcValue command_topic;
        node_.getParam("platform_state_controller/command_topic", command_topic);
        if (command_topic.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("platform_state_controller: 'command_topic' is not a string. (namespace: %s)",
                      node_.getNamespace().c_str());
        }
        command_topic_ = (std::string)command_topic;
        ROS_INFO("platform_state_controller: Velocity Command Topic Name %s", command_topic_);
    }
    else
    {
        ROS_WARN("Parameter 'Command Topic' not set, set the default one: 'platform_state_controller/cmd_vel'");
        command_topic_ = "platform_state_controller/cmd_vel";
    }

    // Get Odom Topic
    if (node_.hasParam("platform_state_controller/odom_topic"))
    {
        XmlRpc::XmlRpcValue odom_topic;
        node_.getParam("platform_state_controller/odom_topic", odom_topic);
        if (odom_topic.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("platform_state_controller: 'odom_topic' is not a string. (namespace: %s)",
                      node_.getNamespace().c_str());
        }
        odom_topic_ = (std::string)odom_topic;
        ROS_INFO("platform_state_controller: Odometry Topic Name %s", odom_topic_);
    }

    else
    {
        ROS_WARN("Parameter 'Odometry Topic' not set, set the default one: 'platform_state_controller/odom'");
        odom_topic_ = "platform_state_controller/odom";
    }

    // Publish Odometry
    if (node_.hasParam("platform_state_controller/publish_odom"))
    {
        XmlRpc::XmlRpcValue publish_odom;
        node_.getParam("platform_state_controller/publish_odom", publish_odom);
        if (publish_odom.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        {
            ROS_ERROR("platform_state_controller: 'publish_odom' is not a boolean. (namespace: %s)",
                      node_.getNamespace().c_str());
        }
        publish_odom_ = (bool)publish_odom;
    }
    else
    {
        ROS_WARN("Parameter 'Publish Odometry' not set, set the default one: 'false'");
    }

    // TODO Broadcast TF
}

std::string platform_state_controller::str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); } // correct
    );
    return s;
}

void platform_state_controller::getCommand(const geometry_msgs::Twist::ConstPtr &msg)
{
    command_velocity_[0] = msg->linear.x;
    if (not(platform_mode_ == "omni"))
        command_velocity_[1] = 0.0;
    else
        command_velocity_[1] = msg->linear.y;
    command_velocity_[2] = msg->angular.z;
}

void platform_state_controller::init_ros_communication()
{
    odom_pub_ = node_.advertise<nav_msgs::Odometry>(odom_topic_, 1000);
    cmd_sub_ = node_.subscribe(command_topic_, 100, &platform_state_controller::getCommand, this);
}

// void platform_state_controller::publishOdometry()
// {
//     // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
//     nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
//     // Header
//     odom->header.stamp = ros::Time::now();
//     odom->header.frame_id = odom_frame_;
//     odom->child_frame_id = base_frame_;
//     // Position
//     odom->pose.pose.position.x = base_pose_[0];
//     odom->pose.pose.position.y = base_pose_[1];
//     odom->pose.pose.position.z = 0.0;
//     odom->pose.pose.orientation = base_orientation_;
//     // Velocity
//     odom->twist.twist.linear.x = base_velocity_[0];
//     odom->twist.twist.linear.y = base_velocity_[1];
//     odom->twist.twist.angular.z = base_velocity_[2];
//     odom_pub_.publish(odom);
// }

void platform_state_controller::eulerToQuatMsg(const double &roll, const double &pitch, const double &yaw, geometry_msgs::Quaternion &msg_quat)
{
    tf::Quaternion quat;
    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(quat, msg_quat);
}

bool platform_state_controller::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{

    // Get Parameters
    platform_state_controller::read_xml_robot_params(node_);
    platform_state_controller::read_xml_controller_params(node_);

    // Init Subscribers and Publishers
    platform_state_controller::init_ros_communication();

    // Resize State Variables
    base_pose_.resize(3);
    base_velocity_.resize(3);
    qnam.resize(joint_names_.size());
    qpos.resize(joint_names_.size());
    qvel.resize(joint_names_.size());
    qeff.resize(joint_names_.size());

    // Initiate Base Orientation
    platform_state_controller::eulerToQuatMsg(0.0, 0.0, 0.0, base_orientation_);

    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        hardware_interface::JointHandle j = robot->getHandle(joint_names_[i]);
        joints_.push_back(j);
    }

    return true;
}

void platform_state_controller::starting(const ros::Time &time)
{

    // Get Time Now
    begin = ros::Time::now().toSec();

    ros::spinOnce();

    std::cout << "Read Variables in Starting" << std::endl;
    for (unsigned int i = 0; i < joints_.size(); i++)
    {
        qnam[i] = joints_[i].getName();
        qpos[i] = joints_[i].getPosition();
        qvel[i] = joints_[i].getVelocity();
        qeff[i] = joints_[i].getEffort();
    }

    // Init Send Command Zeros
    ROS_INFO("Send Zero Commands");
    for (unsigned int i = 0; i < joints_.size(); i++)
        joints_[i].setCommand(0);
}

void platform_state_controller::update(const ros::Time &time, const ros::Duration &duration)
{

    ros::spinOnce();

    // Time Now
    double t = ros::Time::now().toSec() - begin;
    // Loop
    std::cout << "===========================================" << std::endl;

    // Read Joint Variables
    for (unsigned int i = 0; i < joints_.size(); i++)
    {
        qnam[i] = joints_[i].getName();
        qpos[i] = joints_[i].getPosition();
        qvel[i] = joints_[i].getVelocity();
        qeff[i] = joints_[i].getEffort();
    }

    // Get Velcity Commands wrt Robot Frame
    std::cout << "Velocity Commands Read By Topic: \n"
              << "Linear x: " << command_velocity_[0] << ", y: " << command_velocity_[1] << "| Angular z: " << command_velocity_[2] << std::endl;
    double rad_to_deg = 180.0 / 3.14;
    std::cout << "Base Orientation: " << rad_to_deg * qpos[2] << std::endl;
    // Calculate Velocity Commands wrt World Frame
    std::vector<double> vc = {0.0, 0.0, 0.0};
    vc[0] = cos(qpos[2]) * command_velocity_[0] - sin(qpos[2]) * command_velocity_[1];
    vc[1] = sin(qpos[2]) * command_velocity_[0] + cos(qpos[2]) * command_velocity_[1];
    vc[2] = command_velocity_[2];
    std::cout << "Calculated Velocity Commands: \n"
              << "Linear x: " << vc[0] << ", y: " << vc[1] << "| Angular z: " << vc[2] << std::endl;

    // Calculate Odometry if Needed - Integrate Velocity
    for (unsigned int i = 0; i < joints_.size(); i++)
        joints_[i].setCommand(vc[i]);

    std::cout << "===========================================" << std::endl;
}

void platform_state_controller::stopping(const ros::Time &time)
{
    std::cout << "Stopping Platform State Controller" << std::endl;

    // Spin Once To Pub MSGS
    ros::spinOnce();

    for (unsigned int i = 0; i < joints_.size(); i++)
        joints_[i].setCommand(0.0);
}

} // namespace summit_mico_control

PLUGINLIB_EXPORT_CLASS(summit_mico_control::platform_state_controller, controller_interface::ControllerBase)