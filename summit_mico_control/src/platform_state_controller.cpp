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
        std::string ns = _node.getNamespace();

        XmlRpc::XmlRpcValue joint_names;
        if (!node_.getParam(ns + "/joints", joint_names))
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
        if (!node_.getParam(ns + "/mode", platform_mode))
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
            std::cout << "Platform Mode: " << platform_mode_ << std::endl;
        }

        // Get Odom Frame
        if (node_.hasParam(ns + "/odom_frame"))
        {
            XmlRpc::XmlRpcValue odom_frame;
            node_.getParam(ns + "/odom_frame", odom_frame);
            if (odom_frame.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("platform_state_controller: 'odom_frame' is not a string. (namespace: %s)",
                          node_.getNamespace().c_str());
            }
            odom_frame_ = (std::string)odom_frame;
            ROS_INFO("platform_state_controller: Odometry Frame Name:");
            std::cout << odom_frame_ << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'Odometry Frame' not set, set the default one: 'odom'");
            odom_frame_ = "odom";
        }

        // Get Base Robot Frame
        if (node_.hasParam(ns + "/base_frame"))
        {
            XmlRpc::XmlRpcValue base_frame;
            node_.getParam(ns + "/base_frame", base_frame);
            if (base_frame.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("platform_state_controller: 'base_frame' is not a string. (namespace: %s)",
                          node_.getNamespace().c_str());
            }
            base_frame_ = (std::string)base_frame;
            ROS_INFO("platform_state_controller: Base Frame Name");
            std::cout << base_frame_ << std::endl;
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
        std::string ns = _node.getNamespace();
        // Get Command Topic
        if (node_.hasParam(ns + "/command_topic"))
        {
            XmlRpc::XmlRpcValue command_topic;
            node_.getParam(ns + "/command_topic", command_topic);
            if (command_topic.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("platform_state_controller: 'command_topic' is not a string. (namespace: %s)",
                          node_.getNamespace().c_str());
            }
            command_topic_ = (std::string)command_topic;
            ROS_INFO("platform_state_controller: Velocity Command Topic Name");
            std::cout << command_topic_ << std::endl;
        }
        else
        {
            ROS_WARN("Parameter 'Command Topic' not set, set the default one: 'platform_state_controller/cmd_vel'");
            command_topic_ = "platform_state_controller/cmd_vel";
        }

        // Get Odom Topic
        if (node_.hasParam(ns + "/odom_topic"))
        {
            XmlRpc::XmlRpcValue odom_topic;
            node_.getParam(ns + "/odom_topic", odom_topic);
            if (odom_topic.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("platform_state_controller: 'odom_topic' is not a string. (namespace: %s)",
                          node_.getNamespace().c_str());
            }
            odom_topic_ = (std::string)odom_topic;
            ROS_INFO("platform_state_controller: Odometry Topic Name");
            std::cout << odom_topic_ << std::endl;
        }

        else
        {
            ROS_WARN("Parameter 'Odometry Topic' not set, set the default one: 'platform_state_controller/odom'");
            odom_topic_ = ns + "/odom";
        }

        // Publish Odometry
        if (node_.hasParam(ns + "/publish_odom"))
        {
            XmlRpc::XmlRpcValue publish_odom;
            node_.getParam(ns + "/publish_odom", publish_odom);
            if (publish_odom.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
            {
                ROS_ERROR("platform_state_controller: 'publish_odom' is not a boolean. (namespace: %s)",
                          node_.getNamespace().c_str());
            }
            publish_odom_ = (bool)publish_odom;
            ROS_INFO("platform_state_controller: Publish odom topic");
            std::cout << publish_odom_ << std::endl;
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

    double platform_state_controller::constrainAngle(double x)
    {
        x = fmod(x + M_PI, 2 * M_PI);
        if (x < 0)
            x += 2 * M_PI;
        return x - M_PI;
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

    void platform_state_controller::init_ros_communication(const ros::NodeHandle &n)
    {

        odom_pub_ = node_.advertise<nav_msgs::Odometry>(odom_topic_, 1000);
        cmd_sub_ = node_.subscribe(command_topic_, 100, &platform_state_controller::getCommand, this);
    }

    std::string platform_state_controller::getLeafNamespace(const ros::NodeHandle &nh)
    {
        const std::string complete_ns = nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        return complete_ns.substr(id + 1);
    }

    std::string platform_state_controller::getRootNamespace(const ros::NodeHandle &nh)
    {
        const std::string complete_ns = nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        return complete_ns.substr(1, id - 1);
    }

    void platform_state_controller::publishOdometry()
    {
        // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
        nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
        // Header
        odom->header.stamp = ros::Time::now();
        odom->header.frame_id = odom_frame_;
        odom->child_frame_id = base_frame_;
        // Position
        odom->pose.pose.position.x = base_pose_[0];
        odom->pose.pose.position.y = base_pose_[1];
        odom->pose.pose.position.z = 0.0;
        odom->pose.pose.orientation = base_orientation_;
        // Velocity
        odom->twist.twist.linear.x = base_velocity_[0];
        odom->twist.twist.linear.y = base_velocity_[1];
        odom->twist.twist.angular.z = base_velocity_[2];
        odom_pub_.publish(odom);
    }

    void platform_state_controller::eulerToQuatMsg(const double &roll, const double &pitch, const double &yaw, geometry_msgs::Quaternion &msg_quat)
    {
        tf2::Quaternion quat_tf;
        quat_tf.setRPY( roll, pitch, yaw );
        msg_quat = tf2::toMsg(quat_tf);
    }

    bool platform_state_controller::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
    {
        ROS_INFO(" -- Initiate Platform State Controller -- ");

        // Get Parameters
        platform_state_controller::read_xml_robot_params(n);
        platform_state_controller::read_xml_controller_params(n);

        ROS_INFO(" -- Read From Parameter Server -- ");

        // Init Subscribers and Publishers
        platform_state_controller::init_ros_communication(n);

        ROS_INFO(" -- Init Ros Communication -- ");

        // Resize State Variables
        base_pose_.resize(3);
        base_velocity_.resize(3);
        qnam.resize(joint_names_.size());
        qpos.resize(joint_names_.size());
        qvel.resize(joint_names_.size());
        qeff.resize(joint_names_.size());
        command_velocity_.resize(3);

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
        std::cout << "Joint_ size: " << joints_.size() << std::endl;
        for (unsigned int i = 0; i < joints_.size(); i++)
        {
            qnam[i] = joints_[i].getName();
            qpos[i] = joints_[i].getPosition();
            qvel[i] = joints_[i].getVelocity();
            qeff[i] = joints_[i].getEffort();
            std::cout << "qname[" << i << "] : " << qnam[i] << std::endl;
            std::cout << "qpos[" << i << "] : " << qpos[i] << std::endl;
        }

        base_pose_[0] = qpos[0];
        base_pose_[1] = qpos[1];

        base_velocity_[0] = qvel[0];
        base_velocity_[1] = qvel[1];
        base_velocity_[2] = qvel[2];

        platform_state_controller::eulerToQuatMsg(0.0, 0.0, qpos[2], base_orientation_);

        command_velocity_[0] = 0.0;
        command_velocity_[1] = 0.0;
        command_velocity_[2] = 0.0;

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
        // std::cout << "===========================================" << std::endl;

        // Read Joint Variables
        for (unsigned int i = 0; i < joints_.size(); i++)
        {
            qnam[i] = joints_[i].getName();
            qpos[i] = joints_[i].getPosition();
            qvel[i] = joints_[i].getVelocity();
            qeff[i] = joints_[i].getEffort();
        }

        // Get Velcity Commands wrt Robot Frame
        double rad_to_deg = 180.0 / 3.14;
        double theta = platform_state_controller::constrainAngle(qpos[2]);

        // Calculate Velocity Commands wrt World Frame
        std::vector<double> vc = {0.0, 0.0, 0.0};
        vc[0] = cos(theta) * command_velocity_[0] - sin(theta) * command_velocity_[1];
        vc[1] = sin(theta) * command_velocity_[0] + cos(theta) * command_velocity_[1];
        vc[2] = command_velocity_[2];

        if (publish_odom_)
        {
            base_pose_[0] = qpos[0];
            base_pose_[1] = qpos[1];

            base_velocity_[0] = qvel[0];
            base_velocity_[1] = qvel[1];
            base_velocity_[2] = qvel[2];

            platform_state_controller::eulerToQuatMsg(0.0, 0.0, qpos[2], base_orientation_);
            platform_state_controller::publishOdometry();
        }

        // Calculate Odometry if Needed - Integrate Velocity
        for (unsigned int i = 0; i < joints_.size(); i++)
            joints_[i].setCommand(vc[i]);
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