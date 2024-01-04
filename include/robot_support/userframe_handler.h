// User Frame Handler
// -------------------------------
// Description:
//      Robot system user frame handler 
//      Collects information on custom defined user frames from the parameter server.
//      It then structures and sorts the information into the userframe-message type 
//      and enables the collected information to be published and broadcasted as tf2 transform
//
// Version:
//  0.1 - Initial Version
//        [04.01.2024]  -   Jan T. Olsen
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
#ifndef USERFRAME_HANDLER_H       
#define USERFRAME_HANDLER_H

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


// User-Frame-Handler Class
// -------------------------------
/** \brief Robot system user frame handler 
* Collects information on custom defined user frames from the parameter server.
* Structures and sorts the information into the userframe-message type 
* and enables the collected information to be published and broadcasted as tf2 transform
*/
class UserFrameHandler
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Class constructor
        // -------------------------------
        /** \brief User-Frame-Handler class constuctor
        *
        * \param nh     ROS Nodehandle
        */
        UserFrameHandler(
            ros::NodeHandle& nh);


        // Class destructor
        // -------------------------------
        /** \brief User-Frame-Handler class destructor
        */
        ~UserFrameHandler();


        // Publish User-Frames
        // -------------------------------
        /** \brief Publish custom defined user-frames 
        *
        * Iterate through user-frame map and publish each user-frame
        */
        void publishUserFrames();


        // Broadcast User-Frames
        // -------------------------------
        /** \brief Broadcast custom defined user-frames 
        *
        * Iterate through user-frame map and broadcast each user-frame as tf2 transform
        */
        void broadcastUserFrames();


        // Get User-Frame
        // -------------------------------
        /** \brief Get information on custom user-frame.
        *
        * Searches for given user-frame name in user-frame map 
        * and returns the respective user-frame.
        * If user-frame is not found within map, function returns false.
        *
        * \param user_frame Name of user-frame [std::string]
        * \return Function return: Successful: User-Frame [robot_toolbox::UserFrame] / Unsuccessful: false [bool]
        */
        boost::optional<robot_toolbox::UserFrame> getUserFrame(
            const std::string& user_frame);


        // Get User-Frame Map
        // -------------------------------
        /** \brief Get map of defined user-frames 
        *
        * Each user-frame [robot_toolbox::UserFrame] is paired with a name [std::string]
        *
        * \return Return User-Frame-Map [std::map<std::string, robot_toolbox::UserFrame>]
        */
        std::map<std::string, robot_toolbox::UserFrame> getUserFrameMap();


        // Load User-Frames Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom user-frames from the parameter server.
        *
        * Organize and structure the loaded parameters into respective user-frame message-type
        * and store the information in a user-frame map
        *
        * \param param_name     Name of the User-Frame parameters, located on parameter server [std::string]
        */
        bool loadParamUserFrames(
            const std::string& param_name);

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Initialize User-Frame-Handler
        // -------------------------------
        /** \brief Initialize User-Frame-Handler
        */
        void init();


        


        // Load User-Frames Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom user-frames from the parameter server.
        *
        * Organize and structure the loaded parameters into respective user-frame message-type
        * and store the information in a user-frame map
        *
        * \param param_xml      User-Frame parameters [XmlRpc::XmlRpcValue]
        * \param user_frame_map Reference to User-Frame-Map [std::map<std::string, robot_toolbox::UserFrame>]
        */
        bool loadParamUserFrames(
            const XmlRpc::XmlRpcValue& param_xml,
            std::map<std::string, robot_toolbox::UserFrame>& user_frame_map);

        
        // Load User-Frame Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom user-frame from the parameter server.
        *
        * Organize and structure the loaded parameters into respective user-frame message-type
        *
        * \param param_xml User-Frame parameters [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: User-Frame [robot_toolbox::UserFrame] / Unsuccessful: false [bool]
        */
        boost::optional<robot_toolbox::UserFrame> loadParamUserFrame(
            const XmlRpc::XmlRpcValue& param_xml);


        // Validate User-Frame
        // -------------------------------
        /** \brief Validate User-Frame
        *
        * Validate and check if user-frame exists and is available.
        *
        * \param user_frame User-Frame [std::string]
        * \return Function return: Successful/Unsuccessful (true/false) [bool]
        */
        bool validateFrame(
            const std::string& user_frame);


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
        // Class-Name-Prefix for terminal message
        static const std::string CLASS_PREFIX;
        
        // ROS Nodehandle(s)
        // -------------------------------
        ros::NodeHandle nh_;
        
        // User-Frame Map
        std::map<std::string, robot_toolbox::UserFrame> user_frame_map_;

        // ROS Publsiher(s)
        // -------------------------------
        ros::Publisher user_frames_pub_;

        // ROS TF2 Broadcaster
        // -------------------------------
        static tf2_ros::TransformBroadcaster tf2_broadcaster_;

}; // End Class: UserFrameHandler
#endif // USERFRAME_HANDLER_H 