// User Frame Manager
// -------------------------------
// Description:
//      Robot system user frame handler
//      Collects, sorts and structures information on custom defined user frames 
//      obtained from the parameter server. The manager utilizes the 
//      User-frame-context class to store information as a userframe-message type 
//      for each defined user-frame. Provides functionality for accessing, publishing 
//      and broadcasting information on each user-frame present in the system.
//
// Version:
//  0.1 - Initial Version
//        [08.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_support/user_frame_manager.h"

// User-Frame-Manager Class
// -------------------------------

    // Constants
    // -------------------------------
    const std::string UserFrameManager::CLASS_PREFIX = "UserFrameManager::";

    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameManager::UserFrameManager(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        nh_(nh),
        param_name_(param_name)
    {
        // // Initialize Service-Server(s)
        // auto tmp_createUserFrameObject_server_ = nh_.advertiseService("/user_frame_create", &UserFrameManager::createUserFrameObject, this);
        // auto tmp_updateUserFrameObject_server_ = nh_.advertiseService("/user_frame_update", &UserFrameManager::updateUserFrameObject, this);
        // createUserFrameObject_server_ = std::make_shared<ros::ServiceServer>(tmp_createUserFrameObject_server_);
        // updateUserFrameObject_server_ = std::make_shared<ros::ServiceServer>(tmp_updateUserFrameObject_server_);

        // Initialize user-frame manager
        init();
    } // Class Constructor End: UserFrameManager()


    // Class Desctructor
    // -------------------------------
    UserFrameManager::~UserFrameManager()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~UserFrameManager()


    // Initialize User-Frame Manager
    // -------------------------------
    void UserFrameManager::init()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Initializing ... ");

        // User-Frame Parameter(s)
        // -------------------------------
            // Initialize user-frame parameter vector
            param_vec_.clear();

            // Load user-frames parameter data
            if(!loadParamData(param_name_))
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Initialization failed!");

                // Function return
                return;
            }

        // User-Frame Object(s)
        // -------------------------------
            // Iterate over user-frame parameter vector and create user-frame object for each entry
            for(std::size_t i = 0; i < param_vec_.size(); i++)
            {
                // Create user-frame object
                auto userFrameObject = createUserFrameObject(param_vec_[i]);

                // Debug
                userFrameObject->printUserFrame();
            }

        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Initializing finished");
    } // Function End: init()


    // Create User-Frame Object
    // -------------------------------
    // (Function Overloading)
    std::shared_ptr<UserFrameContext> UserFrameManager::createUserFrameObject(
        const robot_toolbox::UserFrame user_frame_data)
    {
        // Create a shared-pointer of a User-Frame Object
        auto userFrameObject = std::make_shared<UserFrameContext>(nh_, user_frame_data);

        // Add user-frame object to user-frame vector
        user_frame_vec_.push_back(userFrameObject);

        // Add user-frame object to user-frame map
        user_frame_map_.insert(std::make_pair(user_frame_data.name, userFrameObject));

        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Successfully created User-Frame Object [" << user_frame_data.name << "]");

        // Function return
        return userFrameObject;
    } // Function End: createUserFrameObject()


    // Create User-Frame Object
    // -------------------------------
    // (Function Overloading)
    std::shared_ptr<UserFrameContext> UserFrameManager::createUserFrameObject(
        const XmlRpc::XmlRpcValue user_frame_param_xml)
    {
        // Create a shared-pointer of a User-Frame Object
        auto userFrameObject = std::make_shared<UserFrameContext>(nh_, user_frame_param_xml);

        // Add user-frame object to user-frame vector
        user_frame_vec_.push_back(userFrameObject);

        // Add user-frame object to user-frame map
        user_frame_map_.insert(std::make_pair(userFrameObject->getUserFrame().name, userFrameObject));
        
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Successfully created User-Frame Object [" << userFrameObject->getUserFrame().name << "]");

        // Function return
        return userFrameObject;
    } // Function End: createUserFrameObject()


    // Get User-Frame Object
    // -------------------------------
    boost::optional<std::shared_ptr<UserFrameContext>> UserFrameManager::getUserFrameObject(
        const std::string& user_frame_name)
    {
        // Define local variable(s)
        std::string map_key = user_frame_name;

        // Search for user-frame in user-frame map
        boost::optional<std::shared_ptr<UserFrameContext>> result_search = Toolbox::Map::searchMapByKey(user_frame_map_, map_key);
        if(!result_search)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! User-Frame [" << user_frame_name <<"] is NOT found in User-Frame-Map");

            // Function return
            return boost::none;
        }

        // Map search success! User-Frame is found in the container
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Successfully retrieved User-Frame [" << user_frame_name << "]");

        // Function return
        return result_search.value();
    } // Function End: getUserFrameObject()


    // Get User-Frame
    // -------------------------------
    std::vector<std::shared_ptr<UserFrameContext>> UserFrameManager::getUserFrameObjectsVec()
    {
        // Return local User-Frame object vector
        return user_frame_vec_;
    }  // Function End: getUserFrameObjectsVec() 


    // Get User-Frame Map
    // -------------------------------
    std::map<std::string, std::shared_ptr<UserFrameContext>> UserFrameManager::getUserObjectsMap()
    {
        // Return local User-Frame object map
        return user_frame_map_;
    } // Function End: getUserObjectsMap() 


    // Publish and Broadcast User-Frames
    // -------------------------------
    void UserFrameManager::publishAndBroadcastUserFrames()
    {
        // Iterate over user-frame map
        for (auto& user_frame : user_frame_map_)
        {
            // Publish and broadcast the current user-frame element
            user_frame.second->publishAndBroadcastUserFrame();
        }
    } // Function End: publishAndBroadcastUserFrame()


    // Update User-Frame Object
    // -------------------------------
    // (Function Overloading)
    void UserFrameManager::updateUserFrameObject(
        UserFrameContext::Ptr& user_frame_oject,
        const robot_toolbox::UserFrame& user_frame_data)
    {
        // Update user-frame object according to given user-frame data
        user_frame_oject->updateUserFrame(user_frame_data);
    }  // Function End: updateUserFrameObject()


    // Update User-Frame Object
    // -------------------------------
    // (Function Overloading)
    void UserFrameManager::updateUserFrameObject(
        const std::string& user_frame_name,
        const robot_toolbox::UserFrame& user_frame_data)
    {
        // Find corresponding user-frame object
        boost::optional<std::shared_ptr<UserFrameContext>> result_search = getUserFrameObject(user_frame_name);
        if(!result_search)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! User-Frame [" << user_frame_name <<"] is NOT found in User-Frame-Map");

            // Function return
            return;
        }

        // Update user-frame object according to given user-frame data
        updateUserFrameObject(result_search.value(), user_frame_data);
    } // Function End: updateUserFrameObject()


    // Load User-Frames Parameter Data
    // -------------------------------
    bool UserFrameManager::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for user-frames parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! User-Frames Parameter(s) [" << param_name << "] not found on parameter server");

            // Function return
            return false;
        }

        // Evaluate parameter(s) type
        // Array: Parameter data contains configuration on multiple user-frames
        if(Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeArray))
        {
            // Iterate over user-frames parameter elements
            for(std::size_t i = 0; i < param_xml.size(); i++)
            {
                // Check if user-frame element is of type struct
                if(!Toolbox::Parameter::checkDataType(param_xml[i], XmlRpc::XmlRpcValue::TypeStruct))
                {
                    // Parameter is not a struct
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! User-Frames Element [" << i << "] is not a struct");

                    // Function return
                    return false;
                }

                // Append user-frame parameter element [i] to local parameter vector
                param_vec_.push_back(param_xml[i]);
            } 
        }

        // Struct: Parameter data contains configuration on a singe user-frame
        else if(Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Append user-frame parameter element to parameter vector
            param_vec_.push_back(param_xml);
        }

        // Unknown: Parameter of user-frame is configured with unknown type
        else
        {
            // Failed to get parameter(s)
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! User-Frames Parameter(s) [" << param_name <<"] is wrongly defined." 
                << " Neither a single- or multi-entry of User-Frames Parameters(s) is found");

            // Function return
            return false;
        }

        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Successfully loaded user-frame parameters [" << param_name << "]");

        // Function return
        return true;
    } // Function End: loadParamData() 


    // // Service Callback: Create User-Frame object
    // // -------------------------------
    // bool UserFrameManager::createUserFrameObjectCB(
    //     robot_toolbox::UserFrameCreate::Request& req,
    //     robot_toolbox::UserFrameCreate::Response& res)
    // {

    // } // Function End: createUserFrameObjectCB() 

    
    // // Service Callback: Update User-Frame object
    // // -------------------------------
    // bool UserFrameManager::updateUserFrameObjectCB(
    //     robot_toolbox::UserFrameUpdate::Request& req,
    //     robot_toolbox::UserFrameUpdate::Response& res)
    // {

    // } // Function End: updateUserFrameObjectCB() 