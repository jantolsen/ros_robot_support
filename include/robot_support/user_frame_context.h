// User Frame Context
// -------------------------------
// Description:
//      Robot system user frame context
//      Collects information on custom defined user frame from the parameter server. 
//      Structures and sorts the information into a user-frame message-type.
//      Provides functionality related to user-frame interaction.
//
// Version:
//  0.1 - Initial Version
//        [05.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef USER_FRAME_CONTEXT_H       
#define USER_FRAME_CONTEXT_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>
    #include <map>

    // Ros
    #include <ros/ros.h>

    // TF2
    #include <tf2_ros/transform_broadcaster.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Toolbox Messages
    #include "robot_toolbox/UserFrame.h"


// User-Frame Context Class
// -------------------------------
/** \brief Robot system user frame context 
* Collects information on custom defined user frame from the parameter server.
* Structures and sorts the information into a user-frame message-type.
* Provides functionality related to user-frame interaction
*/
class UserFrameContext
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Class constructor
        // -------------------------------
        /** \brief User-Frame Context class constuctor
        *
        * \param nh     ROS Nodehandle
        */
        UserFrameContext(
            ros::NodeHandle& nh);


        // Class destructor
        // -------------------------------
        /** \brief User-Frame Context class destructor
        */
        ~UserFrameContext();


        // Publish User-Frame
        // -------------------------------
        /** \brief Publish user-frame. 
        *
        * Publishes custom defined user-frame as a topic.
        * The user-frame is published as a [robot_toolbox::UserFrame] message-type.
        */
        void publishUserFrame();


        // Broadcast User-Frames
        // -------------------------------
        /** \brief Broadcast user-frame.
        *
        * Broadcasts custom defined user-frames as tf2 transforms.
        * The user-frame is broadcasted as a [geometry_msgs::TransformStamped] message-types.
        */
        void broadcastUserFrame();


        // Get User-Frame
        // -------------------------------
        /** \brief Get information on custom user-frame.
        *
        * \return Return User-Frame [robot_toolbox::UserFrame]
        */
        robot_toolbox::UserFrame getUserFrame();

        // Update User-Frame
        // -------------------------------
        /** \brief Update information on custom user-frame.
        *
        * \param user_frame Updated User-Frame [robot_toolbox::UserFrame]
        */
        void updateUserFrame(
            robot_toolbox::UserFrame user_frame);


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Initialize User-Frame Context
        // -------------------------------
        /** \brief Initialize User-Frame Context
        */
        void init();


        // Load Parameter Data
        // -------------------------------
        /** \brief Loads parameter data on custom user-frame from the parameter server.
        *
        * With the loaded parameter data, the function calls "LoadUserFrame" 
        * to organize and structure the loaded parameters into user-frame message-type
        *
        * \param param_name Name of the User-Frame parameter data, located on parameter server [std::string]
        * \return Function return: Successful/Unsuccessful (true/false) [bool]
        */
        bool loadParamData(
            const std::string& param_name);


        // Load User-Frame Data
        // -------------------------------
        /** \brief Reads and loads information on custom user-frames from the parameter server.
        *
        * Organize and structure the loaded parameters into user-frame message-type
        *
        * \param param_xml  User-Frame parameters [XmlRpc::XmlRpcValue]
        * \param user_frame User-Frame [robot_toolbox::UserFrame]
        * \return Function return: Successful/Unsuccessful (true/false) [bool]
        */
        bool loadUserFrame(
            const XmlRpc::XmlRpcValue& param_xml,
            robot_toolbox::UserFrame& user_frame);


        // Validate Frame
        // -------------------------------
        /** \brief Validate Frame
        *
        * Validate and check if frame exists and is available.
        *
        * \param frame Frame name [std::string]
        * \return Function return: Successful/Unsuccessful (true/false) [bool]
        */
        bool validateFrame(
            const std::string& frame);


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
        // Class-Name-Prefix for terminal message
        static const std::string CLASS_PREFIX;
        
        // ROS Nodehandle(s)
        // -------------------------------
        ros::NodeHandle nh_;
        
        // ROS Publsiher(s)
        // -------------------------------
        ros::Publisher user_frame_pub_;

        // ROS TF2 Broadcaster
        // -------------------------------
        static tf2_ros::TransformBroadcaster tf2_broadcaster_;

        // User-Frame
        robot_toolbox::UserFrame user_frame_;

}; // End Class: UserFrameContext
#endif // USER_FRAME_CONTEXT_H 