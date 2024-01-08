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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_support/user_frame_context.h"

// User-Frame-Handler Class
// -------------------------------

    // Constants
    // -------------------------------
    const std::string UserFrameContext::CLASS_PREFIX = "UserFrameContext::";

    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameContext::UserFrameContext(
        ros::NodeHandle& nh,
        const robot_toolbox::UserFrame& user_frame)
    :
        nh_(nh),
        user_frame_(user_frame)
    {
        // Initialize publisher(s)
        user_frame_pub_ = nh_.advertise<robot_toolbox::UserFrame>("/user_frame/" + user_frame_.name, 1);
    } // Class Constructor End: UserFrameHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameContext::UserFrameContext(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        // Constructor delegation
        UserFrameContext(nh, loadParamData(param_name).value())
    {
        // This constructor delegates the construction of the UserFrameContext-class to:
        // UserFrameContext(ros::NodeHandler& nh, const robot_toolbox::UserFrame& user_frame)
    } // Class Constructor End: UserFrameHandler()

    
    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameContext::UserFrameContext(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        // Constructor delegation
        UserFrameContext(nh, loadParamData(param_xml).value())
    {
        // This constructor delegates the construction of the UserFrameContext-class to:
        // UserFrameContext(ros::NodeHandler& nh, const robot_toolbox::UserFrame& user_frame)
    } // Class Constructor End: UserFrameHandler()


    // Class Desctructor
    // -------------------------------
    UserFrameContext::~UserFrameContext()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~UserFrameContext()
    

    // Publish User-Frame
    // -------------------------------
    void UserFrameContext::publishUserFrame()
    {
        // Publish the user-frame
        user_frame_pub_.publish(user_frame_);
    }  // Function End: publishUserFrame() 


    // Broadcast User-Frame
    // -------------------------------
    void UserFrameContext::broadcastUserFrame()
    {
        // Update Transformation time-stamp for the user-frame
        user_frame_.transformStamped.header.stamp = ros::Time::now();

        // Broadcast the user-frame transformation
        tf2_broadcaster_.sendTransform(user_frame_.transformStamped);
    }  // Function End: broadcastUserFrame() 


    // Publish and Broadcast User-Frame
    // -------------------------------
    void UserFrameContext::publishAndBroadcastUserFrame()
    {
        // Publish the local user-frame
        publishUserFrame();
        // Broadcast the local user-frame
        broadcastUserFrame();
    }  // Function End: publishAndBroadcastUserFrame()

    
    // Get User-Frame Message
    // -------------------------------
    robot_toolbox::UserFrame UserFrameContext::getUserFrame()
    {
        // Return local User-Frame
        return user_frame_;
    }  // Function End: getUserFrame()


    // Set User-Frame
    // -------------------------------
    void UserFrameContext::setUserFrame(
        robot_toolbox::UserFrame user_frame)
    {
        // Update local User-Frame
        user_frame_ = user_frame;
    }  // Function End: setUserFrame()


    // Print User-Frame
    // -------------------------------
    void UserFrameContext::printUserFrame()
    {
        // Print information of local user-frame to terminal
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("User-Frame:");
        ROS_INFO_STREAM("--------------------");
        ROS_INFO_STREAM("Name: "        << user_frame_.name);
        ROS_INFO_STREAM("Ref-Frame: "   << user_frame_.ref_frame);
        ROS_INFO_STREAM("   Position:");
        ROS_INFO_STREAM("       x: "    << user_frame_.poseRPY.position.x << " [m]");
        ROS_INFO_STREAM("       y: "    << user_frame_.poseRPY.position.y << " [m]");
        ROS_INFO_STREAM("       z: "    << user_frame_.poseRPY.position.z << " [m]");
        ROS_INFO_STREAM("   Orientation:");
        ROS_INFO_STREAM("       rx: "   << user_frame_.poseRPY.orientation.x << " [deg]");
        ROS_INFO_STREAM("       ry: "   << user_frame_.poseRPY.orientation.y << " [deg]");
        ROS_INFO_STREAM("       rz: "   << user_frame_.poseRPY.orientation.z << " [deg]");
        ROS_INFO_STREAM(" ");
    } // Function End: printUserFrame()
    

    // Load User-Frame Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<robot_toolbox::UserFrame> UserFrameContext::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for User-Frame parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! User-Frame Parameter [" << param_name << "] not found");

            // Function return
            return boost::none;
        }
        // Function return: Call overloading function
        return loadParamData(param_xml);
    } // Function End: loadParamData() 


    // Load User-Frame Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<robot_toolbox::UserFrame> UserFrameContext::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the info-message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        robot_toolbox::UserFrame user_frame;

        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given User-Frame parameter is not a struct");

            // Function return
            return boost::none;
        }

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

        // Validate Reference Frame
        if(!validateFrame(user_frame.ref_frame))
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Reference-Frame [" << user_frame.ref_frame << "]" 
                << " related to User-Frame [" << user_frame.name << "]  is invalid");

            // Function return
            return boost::none;
        } 

        // Assign Transform data of User-Frame
        geometry_msgs::Pose pose = Toolbox::Convert::poseRPYToPose(user_frame.poseRPY);
        user_frame.transformStamped = Toolbox::Convert::poseToTransform(pose, user_frame.ref_frame, user_frame.name);

        // Function return
        return user_frame;
    } // Function End: loadParamData()


    // Validate Frame
    // -------------------------------
    bool UserFrameContext::validateFrame(
        const std::string& frame_name)
    {
        // Create a TF2 buffer
        tf2_ros::Buffer tf_buffer;

        // Create a TF2 listener
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Check if transformation is available
        try
        {
            // Query the TF2 buffer for a transformation
            tf_buffer.canTransform(frame_name, "world", ros::Time(0), ros::Duration(0.5));

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
