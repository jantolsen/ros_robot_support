// Joint State Publisher Node 
// -------------------------------
// Description:
//      This node is applicable for a multi-robot system:
//      Stream the individual motion group joint-state messages for the robot(s)
//      Instead of using the default "joint-state-publisher" (MoveIt)
//      This node will subscribe to the robots controller-joint-states 
//      with related Robot-Prefix and publish a joint-state topic with added robot prefix.
//      This will allow RVIZ to read the joint-states of the robots with their respective prefix name
//
// Version:
//  0.1 - Initial Version
//        [16.10.2022]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    #include <vector>
    #include <tuple>
    
    // Ros
    #include <ros/ros.h>
    #include <ros/callback_queue.h>

    // Message Types
    #include <sensor_msgs/JointState.h>
    #include <std_msgs/String.h>

// Class: Robot
// -------------------------------
// Initializes Subscriber and Publisher for
// Robot-Joint-State with Robot-Prefix
class RobotJointState
{   
    // Private Class members
    // (Accessible only for the class which defines them)
    private:

        // Class Members
        std::string robot_prefix_;
        sensor_msgs::JointState joint_state_;

        ros::Subscriber controller_joint_state_sub_;
        ros::Publisher joint_state_pub_;

        // Controller-Joint-State Callback
        // -------------------------------
        void jointStateCallback(
            const sensor_msgs::JointState::ConstPtr& msg)
        {
            // Assign data from Joint-State Subscriber to Joint-State variable
            joint_state_.header = msg->header;
            joint_state_.name = msg->name;
            joint_state_.position = msg->position;
            joint_state_.velocity = msg->velocity;
            joint_state_.effort = msg->effort;
        } // Function End: jointStateCallback()

    // Public Class members
    // (Accessible for everyone)
    public:

        // Class pointer specifier
        typedef typename std::shared_ptr<RobotJointState> Ptr;

        // Class Constructor
        // -------------------------------
        // (Overloading)
        RobotJointState(
            ros::NodeHandle nh)
        {
            // Temporary variables of subscriber and publisher names
            std::string sub_name = "/controller_joint_states";
            std::string pub_name = "/joint_states";

            // Define Controller-Joint-State Subscriber
            controller_joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>(sub_name, 1e3, &RobotJointState::jointStateCallback, this);

            // Define Joint-State Publisher
            joint_state_pub_ = nh.advertise<sensor_msgs::JointState>(pub_name, 1e3);
        }

        // Class Constructor
        // -------------------------------
        // (Overloading)
        RobotJointState(
            ros::NodeHandle nh,
            const std::string& robot_prefix)
        {
            // Temporary variables of subscriber and publisher names
            std::string sub_name = "/" + robot_prefix + "/controller_joint_states";
            std::string pub_name = "/" + robot_prefix + "/joint_states";

            // Define Controller-Joint-State Subscriber
            controller_joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>(sub_name, 1e3, &RobotJointState::jointStateCallback, this);

            // Define Joint-State Publisher
            joint_state_pub_ = nh.advertise<sensor_msgs::JointState>(pub_name, 1e3);
        }

        // Get Robot Joint-State
        // -------------------------------
        sensor_msgs::JointState getRobotJointState()
        {
            // Return Robot Joint-State
            return joint_state_;
        }

        // Publish Joint-State
        // -------------------------------
        void publishJointState()
        {
            // Publish Joint-State
            // (with Robot-Prefix)
            joint_state_pub_.publish(joint_state_);
        }
};

// Get System Parameters
// -------------------------------
// Search on parameter server for system parameters
// related to Robot-Count and Robot-Prefix
bool getSystemParam(
    const ros::NodeHandle& pnh,
    int& robot_count,
    std::vector<std::string>& robot_prefixes)
{   
    // Defining local variables 
    // -------------------------------
    std::string CLASS_PREFIX = "Joint-State-Publisher-Node::";
    std::string robot_prefix;
    
    // Get Robot Count
    // -------------------------------
    // (Check parameter server for number of robots in the system)
    if(!pnh.getParam("/general/robot_count", robot_count))
    {
        // Robot-Count parameter not found
        // treating system as single-robot configuration
        robot_count = 0;
        
        // Report to terminal
        ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ << ": Warning! Parameter Robot-Count [/general/robot_count] not found");
        ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ << ": Treating system as a single-robot configuration");
    }
    
    // Get Robot Prefixes
    // -------------------------------
    // Multi-Robot System
    if(robot_count > 0)
    {
        // Iterate over robots and get Robot-Prefix for each robot
        for(size_t i = 0; i < robot_count; i++)
        {
            // Get Robot-Prefix for current iteration
            if(!pnh.getParam("/robot_" + std::to_string(i+1) + "/prefix", robot_prefix))
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << ": Failed! Parameter Robot-Prefix [/robot_(" << i << ")/prefix] not found!");
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << ": Is Parameter Robot-Count larger than defined robots?");

                // Function return
                return false;
            }

            // Append Robot-Prefix to vector
            robot_prefixes.push_back(robot_prefix);
        }
    }
    // Single-Robot System
    else
    {
        // Get Robot-Prefix
        if(!pnh.getParam("/robot/prefix", robot_prefix))
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << ": Failed! Parameter Robot-Prefix [/robot/prefix] not found!");

            // Function return
            return false;
        }

        // Append Robot-Prefix to vector
        robot_prefixes.push_back(robot_prefix);
    }

    // Function return
    return true;
} // Function End: getSystemParam()


// Joint State Publisher Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a Anonymous ROS Node with a node name
    ros::init(argc, argv, "joint_state_publisher_node");   

    // Starting the ROS Node by declaring global and private NodeHandle
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 

    // Defining local variables 
    // -------------------------------
    std::string CLASS_PREFIX = "Joint-State-Publisher-Node";
    int robot_count;
    std::vector<std::string> robot_prefixes;
    std::vector<std::shared_ptr<RobotJointState>> robots;
    
    // Publisher
    // -------------------------------
    // Combined Joint-State publisher for all available joint-states of the robots the system
    ros::Publisher joint_state_group_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1e3);  

    // System Parameters
    // -------------------------------
    // Get Robot-Count and Robot-Prefixes
    if(!getSystemParam(pnh, robot_count, robot_prefixes))
    {
        // Report to terminal
        ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << ": Failed!");

        // Function failed
        return -1;
    }

    // Robot(s) in the system
    // -------------------------------
    // Multi-Robot System
    if(robot_count > 0)
    {
        // Create a Robot for each number of Robot(s) in the system
        for(size_t i = 0; i < robot_count; i++)
        {   
            // Declare an instance of Robot-Joint-State class
            auto robot = std::make_shared<RobotJointState>(nh, robot_prefixes[i]);

            // Add the Robot-Class Object to the Robot-Class vector
            robots.push_back(robot);
        }
    }
    // Single-Robot System
    else
    {
        // Empty robot-prefix
        if(robot_prefixes[0].empty())
        {
            // Declare an instance of Robot-Joint-State class
            auto robot = std::make_shared<RobotJointState>(nh);

            // Add the Robot-Class Object to the Robot-Class vector
            robots.push_back(robot);
        }
        // Non-Empty robot-prefix
        else
        {
            // Declare an instance of Robot-Joint-State class
            auto robot = std::make_shared<RobotJointState>(nh, robot_prefixes[0]);

            // Add the Robot-Class Object to the Robot-Class vector
            robots.push_back(robot);
        }
    }

    // ROS-Node Loop
    // -------------------------------
    // (Main Loop for ROS-Node
    while (ros::ok())
    {
        // Declare Joint-States-Vector for containing each Robot-Joint-State 
        std::vector<sensor_msgs::JointState> joint_states;

        // Declare Combined Joint-State containing all Joint-States in the system
        sensor_msgs::JointState joint_state_group;

        // Iterate over each Robot(s) in the system
        for(size_t i = 0; i < robots.size(); i++)
        {   
            // Non-Empty robot prefix
            if(!robot_prefixes[0].empty())
            {   
                // Publish Joint-State for indexed robot
                // (with Robot-prefix)
                robots[i]->publishJointState();
            }

            // Get Joint-State for indexed robot
            sensor_msgs::JointState robot_joint_state = robots[i]->getRobotJointState();

            // Append Robot Joint-State to Joint-State Vector
            joint_states.push_back(robot_joint_state);
        }

        
        // Iterate over each Joint-State in Joint-State vector
        for(int j = 0; j < joint_states.size(); j++)
        {
            // Iterate over each joint of joint-state index
            for(int k = 0; k < joint_states[j].name.size(); k++)
            {
                // Update Combined Joint-States
                joint_state_group.header.seq = joint_state_group.header.seq + 1;
                joint_state_group.header.stamp = ros::Time::now();

                // Check Name Size for index Joint-State [j]
                if (joint_states[j].name.size() > k)
                {
                    // Append Name to combined Joint-States
                    joint_state_group.name.push_back(joint_states[j].name[k]);
                }

                // Check Position Size for index Joint-State [j]
                if (joint_states[j].position.size() > k)
                {
                    // Append Position to combined Joint-States
                    joint_state_group.position.push_back(joint_states[j].position[k]);
                }

                // Check Velocity Size for index Joint-State [j]
                if (joint_states[j].velocity.size() > k)
                {
                    // Append Velocity to combined Joint-States
                    joint_state_group.velocity.push_back(joint_states[j].velocity[k]);
                }

                // Check Effort Size for index Joint-State [j]
                if (joint_states[j].effort.size() > k)
                {
                    // Append Effort to combined Joint-States
                    joint_state_group.effort.push_back(joint_states[j].effort[k]);
                }
            } // End For-Loop: Joint Iteration
        } // End For-Loop: Joint-State Iteration

        // Publish combined Joint-States for all available Joint-State
        // (with Robot-prefix)
        joint_state_group_pub.publish(joint_state_group);

        // ROS Spin
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }
}                                                                      