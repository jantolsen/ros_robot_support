// Test User-Frame Node 
// -------------------------------
// Description:
//      Test User-Frame Node
//
// Version:
//  0.1 - Initial Version
//        [04.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Toolbox Messages
    #include "robot_toolbox/UserFrame.h"

    // User-Frame-Handler
    #include "robot_support/user_frame_manager.h"
    #include "robot_support/user_frame_context.h"


// Test: Function
// -------------------------------
void testFunc(robot_toolbox::UserFrame& userFrameMsg)
{
    // Debug Print
    // -------------------------------
    ROS_INFO(" ");
    ROS_INFO("User-Frames:");
    ROS_INFO("--------------------");
    ROS_INFO_STREAM("Name: " << userFrameMsg.name);
    ROS_INFO_STREAM("Reference-Frame: " << userFrameMsg.ref_frame);
    ROS_INFO_STREAM("   Position:");
    ROS_INFO_STREAM("       x: " << userFrameMsg.poseRPY.position.x);
    ROS_INFO_STREAM("       y: " << userFrameMsg.poseRPY.position.y);
    ROS_INFO_STREAM("       z: " << userFrameMsg.poseRPY.position.z);
    ROS_INFO_STREAM("   Orientation:");
    ROS_INFO_STREAM("       r: " << userFrameMsg.poseRPY.orientation.x);
    ROS_INFO_STREAM("       p: " << userFrameMsg.poseRPY.orientation.y);
    ROS_INFO_STREAM("       y: " << userFrameMsg.poseRPY.orientation.z);
    ROS_INFO(" ");

    

} // Function end: testFunc()


// Info-Kinemaatics Test Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_userframe_handler");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Test(s)
    // -------------------------------
        std::string param_name;
        XmlRpc::XmlRpcValue param_xml;


    // User-Frame Manager
    // -------------------------------
        // Define and initialize User-Frame Manager
        UserFrameManager userFrameManager(nh, "/user_frames");


    // // User-Frame #1
    // // -------------------------------
    //     // Define and initialize User-Frame-Handler
    //     param_name = "/user_frames/test1";
    //     UserFrameContext uf_1(nh, param_name);
    //     // UserFrameContext uf_1(nh, "/user_frames/test1");

    //     // Debug Print
    //     uf_1.printUserFrame();

    // // User-Frame #2
    // // -------------------------------
    //     // Define and initialize User-Frame-Handler
    //     param_name = "/user_frames/test2";
        
    //     // Check parameter server for Information-Kinematics parameters
    //     if(!ros::param::get(param_name, param_xml))
    //     {
    //         // Failed to get parameter
    //         ROS_ERROR_STREAM("Failed! User-Frames Parameter [" << param_name << "] not found");

    //         // Function return
    //         return false;
    //     }

    //     UserFrameContext uf_2(nh, param_xml);

    //     // Debug Print
    //     uf_2.printUserFrame();

    // // User-Frame #3
    // // -------------------------------
    //     // Define User-Frame Msgs
    //     robot_toolbox::UserFrame userFrameMsg;

    //     // Assign parameters
    //     userFrameMsg.name = "test3";
    //     userFrameMsg.ref_frame = "world";
    //     userFrameMsg.poseRPY.position.x = -1.0;
    //     userFrameMsg.poseRPY.position.y = 5.8;
    //     userFrameMsg.poseRPY.position.z = 2.3;
    //     userFrameMsg.poseRPY.orientation.x = -4.3;
    //     userFrameMsg.poseRPY.orientation.y = 8.0;
    //     userFrameMsg.poseRPY.orientation.z = 3.14;

    //     // Assign Transform data of User-Frame
    //     geometry_msgs::Pose pose = Toolbox::Convert::poseRPYToPose(userFrameMsg.poseRPY);
    //     userFrameMsg.transformStamped = Toolbox::Convert::poseToTransform(pose, userFrameMsg.ref_frame, userFrameMsg.name);

    //     // Define and initialize User-Frame-Handler
    //     UserFrameContext uf_3(nh, userFrameMsg);

    //     // Debug Print
    //     uf_3.printUserFrame();


    
    ros::Rate rate(10.0);
    // Run testFunc in a loop
    while (ros::ok()) 
    {
        // // Publish User-Frame
        // uf_1.publishUserFrame();
        // uf_2.publishUserFrame();
        // uf_3.publishUserFrame();

        // // Broadcast User-Frame
        // uf_1.broadcastUserFrame();
        // uf_2.broadcastUserFrame();
        // uf_3.broadcastUserFrame();

        userFrameManager.publishAndBroadcastUserFrames();

        ros::spinOnce(); // Handle ROS callbacks
    }
    
    // // Spin to keep the node alive
    // ros::spin();
    
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}

