// Toolbox Prefix Parameter 
// -------------------------------
// Description:
//      Toolbox for Prefix Paramters
//      Functions for loading and remapping robot specific parameters to correct for robot prefix
//      Applicaple for when having multiple robots in the same scene.
//
// Version:
//  0.1 - Initial Version
//        [10.09.2022]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Main Header-File
    #include "robot_support/prefix_param_tool.h"

// Namespace: Prefix Param Toolbox
// -------------------------------
namespace PrefixParamTool
{
    // Prefix Joint-Names
    // -------------------------------
    void prefixJointNames(
        const ros::NodeHandle& nh,
        const std::string& robot_prefix)
    {   
        // Defining local variables
        std::vector<std::string> joint_names;
        std::vector<std::string> prefix_joint_names;

        // Private Parameters: Joint-Names
        // -------------------------------
        // Check private parameter server for joint-names
        if(nh.getParam("controller_joint_names", joint_names))
        {
            // Iterate over each joint-names
            for (std::size_t i = 0; i < joint_names.size(); i ++)
            {
                // Check for empty Robot-Prefix
                if(robot_prefix.empty())
                {
                    // Keep original joint-name
                    prefix_joint_names.push_back(joint_names[i]); 
                }
                // Non-Empty Robot-Prefix
                else
                {
                    // Add robot-prefix to joint-name
                    std::string joint_name = robot_prefix + "_" + joint_names[i];

                    // Assign robot-prefix to each joint-name
                    prefix_joint_names.push_back(joint_name); 
                }
            }
        }
        // Parameter not found on private parameter server
        else
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Controller Joint-Names Parameter not found!");

            // Function failed
            return;
        }

        // Global Parameters: Joint-Names
        // -------------------------------
        // Check for existing controller joint-names on global parameter server
        if(nh.getParam("/controller_joint_names", joint_names))
        {
            // Iterate over each joint-name
            for (std::size_t i = 0; i < prefix_joint_names.size(); i ++)
            {
                // Append newly assign joint-names to existing list
                joint_names.push_back(prefix_joint_names[i]); 
            }
            
            // Replace new joint-names on global parameter server
            nh.setParam("/controller_joint_names", joint_names);
        }
        // No existing joint-names on global parameter server
        else
        {
            // Create new joint-names on global parameter server
            nh.setParam("/controller_joint_names", prefix_joint_names);
        }

        // Create new robot specific joint-names on global parameter server
        nh.setParam("/" + robot_prefix + "/controller_joint_names", prefix_joint_names);
    } // End-Function: prefixJointNames()
    

    // Prefix Joint-Limits
    // -------------------------------
    void prefixJointLimits(
        const ros::NodeHandle& nh,
        const std::string& robot_prefix)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue joint_limits_xml;
        XmlRpc::XmlRpcValue joint_limit_xml;
        std::vector<std::string> joint_names;
        
        // Joint-Names
        // -------------------------------
        // Check parameter server for existing robot specific controller-joint-names parameter
        if(!nh.getParam("/" + robot_prefix + "/controller_joint_names", joint_names))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Controller Joint-Names for Robot [" << robot_prefix << "] not found");

            // Function failed
            return;
        }

        // Joint-Limits
        // -------------------------------
        // Check private parameter server for controller joint-limits
        if(nh.getParam("joint_limits", joint_limits_xml))
        {
            // Verify that Joint-Limits array equals the size of Joint-Names
            if(joint_limits_xml.size() != joint_names.size())
            {
                // Report to terminal
                ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Controller Joint-Names and -Limits are not same size!");

                // Function failed
                return;
            }

            // Iterate over each joint of Joint-Limits
            for (std::size_t i = 0; i < joint_limits_xml.size(); i ++)
            {
                // Get Joint-Limit for current index
                nh.getParam("joint_limits/joint_" + std::to_string(i + 1), joint_limit_xml);
                    // Parameter Server's Joint-Limits are defined as a XmlRpc-type
                    // (size = 6, starting at 0-index) hence (i + 1)

                // Create new Joint-Limit Parameter with robot prefix
                nh.setParam("/robot_description_planning/joint_limits/" + joint_names[i], joint_limit_xml);
            }
        }
        // Parameter not found on parameter server
        else
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Controller Joint-Limits Parameter not found!");

            // Function failed
            return;
        }
    } // End-Function: prefixJointLimits()


    // Prefix Controller-List
    // -------------------------------
    void prefixControllerList(
        const ros::NodeHandle& nh,
        const std::string& robot_prefix)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue controller_list_xml;
        XmlRpc::XmlRpcValue robot_controller_xml;
        XmlRpc::XmlRpcValue joint_names_xml;

        // Joint-Names
        // -------------------------------
        // Check parameter server for existing robot specific controller-joint-names parameter
        if(!nh.getParam("/" + robot_prefix + "/controller_joint_names", joint_names_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Controller Joint-Names for Robot [" << robot_prefix << "] not found");

            // Function failed
            return;
        }

        // Controller-List
        // -------------------------------
        // Define robot specific Robot-Controller parameters
        robot_controller_xml["name"] = robot_prefix;
        robot_controller_xml["action_ns"] = "joint_trajectory_action";
        robot_controller_xml["type"] = "FollowJointTrajectory";
        robot_controller_xml["joints"] = joint_names_xml;

        // Check for existing controller-list on global parameter server
        if(nh.getParam("/move_group/controller_list", controller_list_xml))
        {
            // Set robot index equal to the number of independent robots found within the controller-list parameter
            int robot_index = controller_list_xml.size();

            // Append the new robot-controller parameters to the existing controller-list parameter
            controller_list_xml[robot_index] = robot_controller_xml;
            // controller_list[robot_index + 1] = tool_controller;

            // Create new joint-names on global parameter server
            nh.setParam("/move_group/controller_list", controller_list_xml);
        }
        // No existing controller-list on global parameter server
        else
        {
            // Append the new robot-controller parameters to the controller-list parameter
            controller_list_xml[0] = robot_controller_xml;

            // Create new controller-list parameters on global parameter server
            nh.setParam("/move_group/controller_list", controller_list_xml);
        }
    } // End-Function: prefixControllerList()


    // Prefix Topic-List
    // -------------------------------
    void prefixTopicList(
        const ros::NodeHandle& nh,
        const std::string& robot_prefix,
        const std::string& robot_type)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue topic_list_xml;
        XmlRpc::XmlRpcValue robot_topic_xml;
        XmlRpc::XmlRpcValue joint_names_xml;

        // Joint-Names
        // -------------------------------
        // Check parameter server for existing robot specific controller-joint-names parameter
        if(!nh.getParam("/" + robot_prefix + "/controller_joint_names", joint_names_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Controller Joint-Names for Robot [" << robot_prefix << "] not found");

            // Function failed
            return;
        }
        
        // Topic-List
        // -------------------------------
        // Define robot specific Robot-Controller parameters
        robot_topic_xml["name"] = robot_type + "_controller";
        robot_topic_xml["ns"] = robot_prefix;
        robot_topic_xml["group"] = 0;
        robot_topic_xml["joints"] = joint_names_xml;

        // Append the new robot-controller parameters to the controller-list parameter
        topic_list_xml[0] = robot_topic_xml;

        // Create new topic-list parameters on global parameter server
        nh.setParam("/" + robot_prefix + "/topic_list", topic_list_xml);
    } // End-Function: prefixTopicList()


    // Prefix Cartesian-Limits Parameters
    // -------------------------------
    void prefixCartesianLimits(
        const ros::NodeHandle& nh,
        const std::string& robot_prefix)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue cartesian_limits_xml;
        
        // Check parameter server for existing cartesian-limits parameters
        if(!nh.getParam("cartesian_limits", cartesian_limits_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Cartesian-Limits Parameters not found!");

            // Function failed
            return;
        }

        // Create new cartesian-limits parameters on global parameter server
        nh.setParam("/robot_description_planning/cartesian_limits/", cartesian_limits_xml);
    } // End-Function: prefixCartesianLimits()


    // Prefix Kinematics Parameters
    // -------------------------------
    void prefixKinematicsParam(
        const ros::NodeHandle& nh,
        const std::string& robot_prefix,
        const std::string& robot_type)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue kdl_kinematics_xml;
        XmlRpc::XmlRpcValue tracik_kinematics_xml;
        XmlRpc::XmlRpcValue opw_kinematics_xml;
        XmlRpc::XmlRpcValue lma_kinematics_xml;
        XmlRpc::XmlRpcValue cached_kdl_kinematics_xml;
        XmlRpc::XmlRpcValue cached_tracik_kinematics_xml;
        XmlRpc::XmlRpcValue kinematics_param_xml;
        XmlRpc::XmlRpcValue opw_param_xml;
        XmlRpc::XmlRpcValue kinematics_manipulator_xml;
        XmlRpc::XmlRpcValue kinematics_eoat_xml;
        std::string kinematic_solver;
        
        // Get Default Kinematics Parameters
        // -------------------------------
        // KDL
        if(!nh.getParam("kdl_kinematics", kdl_kinematics_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters [KDL] not found!");

            // Function failed
            return;
        }

        // Trac IK
        if(!nh.getParam("tracik_kinematics", tracik_kinematics_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters [TracIK] not found!");

            // Function failed
            return;
        }

        // OPW
        if(!nh.getParam("opw_kinematics", opw_kinematics_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters [OPW] not found!");

            // Function failed
            return;
        }

        // LMA
        if(!nh.getParam("lma_kinematics", lma_kinematics_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters [LMA] not found!");

            // Function failed
            return;
        }

        // Cached KDL
        if(!nh.getParam("cached_kdl_kinematics", cached_kdl_kinematics_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters [Cached KDL] not found!");

            // Function failed
            return;
        }

        // Cached TracIK
        if(!nh.getParam("cached_tracik_kinematics", cached_tracik_kinematics_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters [Cached TracIK] not found!");

            // Function failed
            return;
        }

        // Get Kinematics Parameters
        // -------------------------------
        // Check parameter server for kinematics parameters
        if(!nh.getParam("/general/kinematics", kinematics_param_xml))
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Kinematics Parameters not found!");

            // Function failed
            return;
        }

        // Determine Kinematic-Solver
        // -------------------------------
        // KDL
        if(kinematics_param_xml["solver"] == "KDL")
        {
            // Assign Kinematics
            kinematics_manipulator_xml = kdl_kinematics_xml;
            kinematics_eoat_xml = kdl_kinematics_xml;
        }
        // TracIK
        else if (kinematics_param_xml["solver"] == "TRACIK")
        {
            // Assign Kinematics
            kinematics_manipulator_xml = tracik_kinematics_xml;
            kinematics_eoat_xml = tracik_kinematics_xml;
        }
        // OPW
        else if (kinematics_param_xml["solver"] == "OPW")
        {
            // Assign Kinematics
            kinematics_manipulator_xml = opw_kinematics_xml;
            kinematics_eoat_xml = opw_kinematics_xml;

            // Get additional OPW Kinematics-Parameters for respective robot-type
            if(!nh.getParam("opw_kinematics_" + robot_type, opw_param_xml))
            {
                // Report to terminal
                ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! OPW Parameters not found for Robot-Type [" << robot_type << "]");

                // Function failed
                return;
            }

            // Add additional OPW Kinematics-Parameters
            // Kinematics Manipulator
            kinematics_manipulator_xml["opw_kinematics_geometric_parameters"] = opw_param_xml["opw_kinematics_geometric_parameters"];
            kinematics_manipulator_xml["opw_kinematics_joint_offsets"] = opw_param_xml["opw_kinematics_joint_offsets"];
            kinematics_manipulator_xml["opw_kinematics_joint_sign_corrections"] = opw_param_xml["opw_kinematics_joint_sign_corrections"];

            // Kinematics EOAT
            kinematics_eoat_xml["opw_kinematics_geometric_parameters"] = opw_param_xml["opw_kinematics_geometric_parameters"];
            kinematics_eoat_xml["opw_kinematics_joint_offsets"] = opw_param_xml["opw_kinematics_joint_offsets"];
            kinematics_eoat_xml["opw_kinematics_joint_sign_corrections"] = opw_param_xml["opw_kinematics_joint_sign_corrections"];

            // // Additional calculation are required for the EOAT-group kinematic parameters
            // geometry_msgs::Transform transform;
            // std::string elbow_link = robot_prefix + "_link_3";
            // std::string eoat_tcp = robot_prefix + "_eoat_tcp";

            // // Acquire relative transformation between elbow-link and eoat-tcp
            // Toolbox::Kinematics::getCurrentTransform(eoat_tcp, elbow_link, transform, true);

            // kinematics_eoat_xml["opw_kinematics_geometric_parameters"]["c4"] = 0.200 + 0.260;
        }
        // LMA
        else if (kinematics_param_xml["solver"] == "LMA")
        {
            // Assign Kinematics
            kinematics_manipulator_xml = lma_kinematics_xml;
            kinematics_eoat_xml = lma_kinematics_xml;
        }
        // Cached KDL
        else if (kinematics_param_xml["solver"] == "CACHED_KDL")
        {
            // Assign Kinematics
            kinematics_manipulator_xml = cached_kdl_kinematics_xml;
            kinematics_eoat_xml = cached_kdl_kinematics_xml;
        }
        // Cached TracIK
        else if (kinematics_param_xml["solver"] == "CACHED_TRACIK")
        {
            // Assign Kinematics
            kinematics_manipulator_xml = cached_tracik_kinematics_xml;
            kinematics_eoat_xml = cached_tracik_kinematics_xml;
        }
        // Unknown
        else
        {
            // Report to terminal
            ROS_ERROR_STREAM(__FUNCTION__ << ": Failed! Unknown Kinematics-Solver");

            // Function failed
            return; 
        }
        
        // Update Global Kinematic-Parameters
        // -------------------------------
        // Update Kinematics with parameters from general settings
        kinematics_manipulator_xml["kinematics_solver_search_resolution"] = kinematics_param_xml["search_resolution"];
        kinematics_manipulator_xml["kinematics_solver_timeout"] = kinematics_param_xml["timeout"];
        kinematics_manipulator_xml["kinematics_solver_attempts"] = kinematics_param_xml["attempts"];

        kinematics_eoat_xml["kinematics_solver_search_resolution"] = kinematics_param_xml["search_resolution"];
        kinematics_eoat_xml["kinematics_solver_timeout"] = kinematics_param_xml["timeout"];
        kinematics_eoat_xml["kinematics_solver_attempts"] = kinematics_param_xml["attempts"];

        // Check for empty Robot-Prefix
        if(robot_prefix.empty())
        {
            // Create new kinematics parameters on global parameter server
            nh.setParam("/robot_description_kinematics/" + robot_prefix + "manipulator", kinematics_manipulator_xml);

            // Create new kinematics parameters on global parameter server
            nh.setParam("/robot_description_kinematics/" + robot_prefix + "eoat", kinematics_eoat_xml);
        }
        // Non-Empty Robot-Prefix
        else
        {
            // Create new kinematics parameters on global parameter server
            nh.setParam("/robot_description_kinematics/" + robot_prefix + "_manipulator", kinematics_manipulator_xml);

            // Create new kinematics parameters on global parameter server
            nh.setParam("/robot_description_kinematics/" + robot_prefix + "_eoat", kinematics_eoat_xml);
        }
    } // End-Function: prefixKinematicsParam()
} // End Namespace