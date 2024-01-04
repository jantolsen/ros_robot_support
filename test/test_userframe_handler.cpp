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
    #include "robot_support/userframe_handler.h"


// Test: Function
// -------------------------------
void testFunc(ros::NodeHandle nh)
{
    // Define User-Frame Msgs
    robot_toolbox::UserFrame userFrameMsg;

    // Define and initialize User-Frame-Handler
    UserFrameHandler userFrameHandler(nh);

    // Load-Param
    userFrameHandler.loadParamUserFrames("/user_frames");

    // Get User-Frame
    auto result = userFrameHandler.getUserFrame("test2");
    if(result)
    {
        userFrameMsg = result.value();
    }
    else
    {
        ROS_ERROR("User-Frame not found!");
    }
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
    testFunc(nh);
        
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    // ros::waitForShutdown();

    // Function return
    return 0;
}

