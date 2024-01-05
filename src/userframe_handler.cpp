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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_support/userframe_handler.h"

// User-Frame-Handler Class
// -------------------------------

    // Constants
    // -------------------------------
    const std::string UserFrameHandler::CLASS_PREFIX = "UserFrameHandler::";


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameHandler::UserFrameHandler(
        ros::NodeHandle& nh)
    :
        nh_(nh)
    {
        // Initialize User-Frame-Handler
        init();
    } // Class Constructor End: UserFrameHandler()


    // Class Desctructor
    // -------------------------------
    UserFrameHandler::~UserFrameHandler()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~UserFrameHandler()


    // Initialize User-Frame-Handler
    // -------------------------------
    void UserFrameHandler::init()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Initializing class");

    } // Function End: init()


    // Publish User-Frames
    // -------------------------------
    void UserFrameHandler::publishUserFrames()
    {

    }  // Function End: publishUserFrames() 


    // Broadcast User-Frames
    // -------------------------------
    void UserFrameHandler::broadcastUserFrames()
    {
        // Create a TF2 Broadcaster
        tf2_ros::TransformBroadcaster broadcast;
        
        // Iterate over user-frame map
        for (auto& user_frame : user_frames_map_)
        {
            // Update Transformation time-stamp for the user-frame element
            user_frame.second.transformStamped.header.stamp = ros::Time::now();

            // Broadcast the user-frame element
            broadcast.sendTransform(user_frame.second.transformStamped);
        }
    }  // Function End: broadcastUserFrames() 


    // Get User-Frame
    // -------------------------------
    boost::optional<robot_toolbox::UserFrame> UserFrameHandler::getUserFrame(
        const std::string& user_frame)
    {
        // Define local variable(s)
        std::string user_frame_key = user_frame;

        // Search for user-frame in user-frame map
        boost::optional<robot_toolbox::UserFrame> result_search = Toolbox::Map::searchMapByKey(user_frames_map_, user_frame_key);
        if(!result_search)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! User-Frame [" << user_frame <<"] is NOT found in User-Frame-Map");

            // Function return
            return boost::none;
        }

        // Map search success! User-Frame is found in the container
        return result_search.value();
    }  // Function End: getUserFrame() 


    // Get User-Frame
    // -------------------------------
    std::vector<robot_toolbox::UserFrame> UserFrameHandler::getUserFrames()
    {
        // Return local User-Frames-Vector
        return user_frames_vec_;
    }  // Function End: getUserFrames() 


    // Get User-Frame Map
    // -------------------------------
    std::map<std::string, robot_toolbox::UserFrame> UserFrameHandler::getUserFramesMap()
    {
        // Return local User-Frame-Map
        return user_frames_map_;
    } // Function End: getUserFramesMap() 


    // Load User-Frames Data
    // -------------------------------
    bool UserFrameHandler::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for Information-Kinematics parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! User-Frames Parameter [" << param_name << "] not found");

            // Function return
            return false;
        }
        // Function return: Call overloading function
        return loadUserFrames(param_xml, user_frames_map_, user_frames_vec_);
    } // Function End: loadParamUserFrames() 


    // Load User-Frames Data
    // -------------------------------
    bool UserFrameHandler::loadUserFrames(
        const XmlRpc::XmlRpcValue& param_xml,
        std::map<std::string, robot_toolbox::UserFrame>& user_frames_map,
        std::vector<robot_toolbox::UserFrame>& user_frames_vec)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue user_frame;

        // Check if parameter is an array
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeArray))
        {
            // Parameter is not an array
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_xml << "] is not an array");

            // Function return
            return false;
        }

        // Iterate over each user-frame
        for (std::size_t i = 0; i < param_xml.size(); i ++)
        {
            // Check if parameter is a struct
            if(!Toolbox::Parameter::checkDataType(param_xml[i], XmlRpc::XmlRpcValue::TypeStruct))
            {
                // Parameter is not a struct
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_xml[i] << "] is not a struct");

                // Function return
                return false;
            }

            // Load user-frame data
            boost::optional<robot_toolbox::UserFrame> result_load = loadUserFrame(param_xml[i]);
            if(!result_load)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! User-Frame [" << param_xml[i] <<"] is NOT loaded");

                // Function return
                return false;
            }

            // Add user-frame to user-frame map and user-frame vector
            user_frames_map.insert(std::pair<std::string, robot_toolbox::UserFrame>(result_load.value().name, result_load.value()));
            user_frames_vec.push_back(result_load.value());
        }

        // Function return
        return true;
    } // Function End: loadParamUserFrames() 


    // Load User-Frame Data
    // -------------------------------
    boost::optional<robot_toolbox::UserFrame> UserFrameHandler::loadUserFrame(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the info-message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        robot_toolbox::UserFrame user_frame;

        // Initialize a flag to track the validation of the parameter loading
        bool params_valid = true;

        // Load, validate and assign parameter data
        if (!Toolbox::Parameter::loadParamData<std::string>(user_frame.name, param_xml, "name")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<std::string>(user_frame.ref_frame, param_xml, "ref_frame")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame.poseRPY.position.x, param_xml["pose"]["position"], "x")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame.poseRPY.position.y, param_xml["pose"]["position"], "y")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame.poseRPY.position.z, param_xml["pose"]["position"], "z")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame.poseRPY.orientation.x, param_xml["pose"]["orientation"], "rx")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame.poseRPY.orientation.y, param_xml["pose"]["orientation"], "ry")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame.poseRPY.orientation.z, param_xml["pose"]["orientation"], "rz")) params_valid =  false;

        // Validate Reference Frame
        if(!validateFrame(user_frame.ref_frame)) params_valid =  false;
        
        // Check if parameter loading was successful
        // (If any parameter failed to load, the flag will be false. Otherwise, it will be true)
        if(!params_valid)
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to User-Frame [" << user_frame.name << "]  is either missing or configured incorrectly");

            // Function return
            return boost::none;
        }

        // Assign Transform data of User-Frame
        geometry_msgs::Pose pose = Toolbox::Convert::poseRPYToPose(user_frame.poseRPY);
        user_frame.transformStamped = Toolbox::Convert::poseToTransform(pose, user_frame.ref_frame, user_frame.name);

        // Function return
        return user_frame;
    } // Function End: loadParamUserFrame() 


    // Validate User-Frame
    // -------------------------------
    bool UserFrameHandler::validateFrame(
        const std::string& user_frame)
    {
        // Create a TF2 buffer
        tf2_ros::Buffer tf_buffer;

        // Create a TF2 listener
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Check if transformation is available
        try
        {
            // Query the TF2 buffer for a transformation
            tf_buffer.canTransform(user_frame, "world", ros::Time(0), ros::Duration(0.5));

            // Fucntion return
            return true;
        }
        // Catch exception(s)
        catch(tf2::TransformException& ex)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Tranformation failed!: " << ex.what());

            // Function return
            return false;
        }
    } // Function End: validateFrame()
