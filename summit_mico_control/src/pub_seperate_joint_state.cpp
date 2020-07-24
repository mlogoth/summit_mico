#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>

/*
 * READ FROM THE PARAMETER SERVER
 */

void read_yaml_params()
{
}

class pubJointState
{
public:
    pubJointState(ros::NodeHandle &_nh, std::vector<std::string> &_joint_names, std::string _ns, std::string _joint_state_topic = "joint_states")
    {
        if (_joint_names.size() > 0)
        {
            joint_names = _joint_names;
            ROS_INFO("Get Joint Names:");
            for (unsigned int i = 0; i < joint_names.size(); i++)
                std::cout << joint_names[i] << std::endl;
        }
        else
        {
            ROS_ERROR("Given Joints vector is empty!");
            exit(-1);
        }

        // Create Publisher and Subscriber
        pub = _nh.advertise<sensor_msgs::JointState>(_ns + "/joint_states", 1000);
        sub = _nh.subscribe(_joint_state_topic, 1000, &pubJointState::getJointStates, this);
    }

private:
    void getJointStates(const sensor_msgs::JointState msg)
    {
        // create new msg
        sensor_msgs::JointState new_msg;
        new_msg.header = msg.header;

        // std::cout<<"msg joint_names:\n"<< msg.name.data()<<std::endl;
        // std::cout<<"node joints:\n"<< joint_names.data() <<std::endl;

        // find the desired joints
        for (auto &joint : joint_names)
        {
            int index = -1;
            auto it = std::find(msg.name.begin(), msg.name.end(), joint);
            // std::cout << "it: "<< it << std::endl;
            if (it != msg.name.end())
            {
                index = std::distance(msg.name.begin(), it);
            }
            else
            {
                //ROS_WARN("Joint Name does not exist");
                //std::cout << joint << std::endl;
                continue;
            }
            // push back the desired joint parameters
            new_msg.name.push_back(msg.name[index]);
            new_msg.velocity.push_back(msg.velocity[index]);
            new_msg.position.push_back(msg.position[index]);
            new_msg.effort.push_back(msg.effort[index]);
        }

        //publish new joint state topic
        pub.publish(new_msg);
    }

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    std::vector<std::string> joint_names;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pub_seperate_joint_state");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    std::vector<std::string> robot_1_joint_names;
    std::string joint_state_topic, _namespace;

    // Get Joint Names
    XmlRpc::XmlRpcValue joint_names;
    if (!n.getParam("pub_seperate_joint_state/joints", joint_names))
    {
        ROS_ERROR("No 'joints' defined in pub_seperate_joint_state node.");
    }

    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("'joints' is not a struct. (node: pub_seperate_joint_state)");
    }

    for (int i = 0; i < joint_names.size(); i++)
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("joints are not strings.");
            //return false;
        }
        robot_1_joint_names.push_back((std::string)name_value);
        std::cout << robot_1_joint_names[i] << std::endl;
    }

    // Get Topic
    if (n.hasParam("pub_seperate_joint_state/joint_states_topic"))
    {
        n.getParam("pub_seperate_joint_state/joint_states_topic", joint_state_topic);
        std::cout << "joint_states_topic: " << joint_state_topic << std::endl;
    }
    else
    {
        ROS_WARN("Parameter 'pub_seperate_joint_state/joint_states_topic' is not set, default value: joint_states");
        joint_state_topic = "joint_states";
    }

    // Get Name Space
    if (n.hasParam("pub_seperate_joint_state/namespace"))
    {
        n.getParam("pub_seperate_joint_state/namespace", _namespace);
        std::cout << "namespace: " << _namespace << std::endl;
    }
    else
    {
        ROS_WARN("Parameter 'pub_seperate_joint_state/namespace' is not set, default value: '' ");
        _namespace = "";
    }
    

    pubJointState robot_1(n, robot_1_joint_names, _namespace,joint_state_topic);

    ROS_INFO("Publish Seperate Joint States Initialization Complete.");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
