// Prefix Parameter Node 
// -------------------------------
// Description:
//      Loads and remaps robot specific parameters to correct for robot prefix.
//      Applicaple for when having multiple robots in the same scene.
//      The node finds the robot's respective parameter yaml-file(s),
//      adds the prefix name, and loads them as parameters to the parameter server
//
// Version:
//  0.1 - Initial Version
//        [10.09.2022]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    #include <vector>
    
    // Ros
    #include <ros/ros.h>

    // Robot Support
    #include "robot_support/prefix_param_tool.h"

// Prefix Parameter Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a Anonymous ROS Node with a node name
    ros::init(argc, argv, "prefix_parameter_node", ros::init_options::AnonymousName);   
    
    // Starting the ROS Node by declaring private NodeHandle
    ros::NodeHandle nh("~"); 

    // Defining local members
    std::string FUNC_PREFIX = "Prefix-Param-Node";
    std::string robot_prefix;
    std::string robot_type;

    // Get parameters loaded as arguments together with node
    // -------------------------------
    // Get Robot-Prefix parameter 
    if(!nh.getParam("robot_prefix", robot_prefix))
    {
        // Failed to get parameter
        ROS_ERROR_STREAM(FUNC_PREFIX <<  ": Failed! Robot-Prefix Parameter not found");

        // Function return
        return false;
    }

    // Get Robot-Type parameter 
    if(!nh.getParam("robot_type", robot_type))
    {
        // Failed to get parameter
        ROS_ERROR_STREAM(FUNC_PREFIX <<  ": Failed! Robot-Type Parameter not found");

        // Function return
        return false;
    }

    // Prefix Robot Joint-Names Parameters
    // -------------------------------
    // Get Joint-Name parameter (loaded together with node)
    // and assigned them according to robot prefix
    PrefixParamTool::prefixJointNames(nh, robot_prefix);

    // Prefix Robot Joint-Limits Parameters
    // -------------------------------
    // Get Joint-Limits parameter (loaded together with node)
    // and assigned them according to robot prefix
    PrefixParamTool::prefixJointLimits(nh, robot_prefix);

    // Prefix Robot Controller-List Parameters
    // -------------------------------
    // Create Controller-List parameters
    // and assigned them according to robot prefix
    PrefixParamTool::prefixControllerList(nh, robot_prefix);

    // Prefix Robot Topic-List Parameters
    // -------------------------------
    // Create Topic-List parameters
    // and assigned them according to robot prefix
    PrefixParamTool::prefixTopicList(nh, robot_prefix, robot_type);

    // Prefix Robot Cartesian-Limits Parameters
    // -------------------------------
    // Get Cartesian-Limit parameter (loaded together with node)
    // and assigned them according to robot prefix
    PrefixParamTool::prefixCartesianLimits(nh, robot_prefix);

    // Prefix Robot Kinematic Parameters
    // -------------------------------
    // Get Kinematic parameter (loaded together with node)
    // and assigned them according to robot prefix
    PrefixParamTool::prefixKinematicsParam(nh, robot_prefix, robot_type);

    // Prefix Robot OMPL Parameters
    // -------------------------------
    // Get OMPL parameter (loaded together with node)
    // and assigned them according to robot prefix
    PrefixParamTool::prefixOMPLParam(nh, robot_prefix);

    // Cleanup
    // -------------------------------
    // Remove private parameters on the anonymous nodehandle
    nh.deleteParam("robot_prefix");
    nh.deleteParam("robot_type");
    nh.deleteParam("controller_joint_names");
    nh.deleteParam("joint_limits");
    nh.deleteParam("cartesian_limits");
    nh.deleteParam("kdl_kinematics");
    nh.deleteParam("tracik_kinematics");
    nh.deleteParam("opw_kinematics");
    nh.deleteParam("cached_kdl_kinematics");
    nh.deleteParam("cached_tracik_kinematics");
    nh.deleteParam("opw_kinematics_" + robot_type);
    nh.deleteParam("move_group/planning_pipelines/ompl/manipulator");
} // Function End: main()