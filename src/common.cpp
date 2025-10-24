/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = "";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    subImgLMsgName = "/camera/camera/infra1/image_rect_raw";
    subImgRMsgName = "/camera/camera/infra2/image_rect_raw";
    subDptMsgName = "/camera/camera/depth/image_rect_raw";
    subRGBMsgName = "/camera/camera/infra1/image_rect_raw";
    subImuAccMsgName = "/camera/camera/accel/sample";
    subImuGyrMsgName = "/camera/camera/gyro/sample";

    // experimentConfig = "/home/ud/IISC/vo/ORB_SLAM3/Examples/RGB-D/RealSense_D435i";
    experimentConfig = "/home/ud/IISC/vo/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D435i";
    initializeVSLAM(experimentConfig);

    //* subscribe to python node to receive settings
    // expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    // configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    if(sensorType == ORB_SLAM3::System::MONOCULAR || sensorType == ORB_SLAM3::System::MONOCULAR){
        subImgMsg_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
        if(sensorType == ORB_SLAM3::System::IMU_MONOCULAR){
            // subImuAccelMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuAccelMsgName, 10, std::bind(&MonocularMode::Imu_accel_callback, this, _1));
            // subImuGyroMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuGyroMsgName, 10, std::bind(&MonocularMode::Imu_gyro_callback, this, _1));
            imu_acc_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuAccMsgName.c_str(), rmw_qos_profile_sensor_data);
            imu_gyr_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuGyrMsgName.c_str(), rmw_qos_profile_sensor_data);
            using SyncPolicy_imu = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>;
            sync_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_imu>>(SyncPolicy_imu(10), *imu_acc_sub_filtered_, *imu_gyr_sub_filtered_);
            sync_imu_->registerCallback(std::bind(&MonocularMode::synchronizedImuCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
    }

    //* subscribe to receive the timestep
    // subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //Stereo topics
    if(sensorType == ORB_SLAM3::System::STEREO || sensorType == ORB_SLAM3::System::IMU_STEREO){
        img1_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, subImgLMsgName.c_str(), rmw_qos_profile_sensor_data);
        img2_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, subImgRMsgName.c_str(), rmw_qos_profile_sensor_data);
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *img1_sub_filtered_, *img2_sub_filtered_);
        sync_->registerCallback(std::bind(&MonocularMode::synchronizedCallback, this, std::placeholders::_1, std::placeholders::_2));
        // subImgRMsg_ = this->create_subscription<sensor_msgs::msg::Image>(subImgRMsgName, 10, std::bind(&MonocularMode::ImgR_callback, this, _1));
        // subImgLMsg_ = this->create_subscription<sensor_msgs::msg::Image>(subImgLMsgName, 1, std::bind(&MonocularMode::ImgL_callback, this, _1));
        if(sensorType == ORB_SLAM3::System::IMU_STEREO){
            imu_acc_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuAccMsgName.c_str(), rmw_qos_profile_sensor_data);
            imu_gyr_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuGyrMsgName.c_str(), rmw_qos_profile_sensor_data);
            using SyncPolicy_imu = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>;
            sync_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_imu>>(SyncPolicy_imu(10), *imu_acc_sub_filtered_, *imu_gyr_sub_filtered_);
            sync_imu_->registerCallback(std::bind(&MonocularMode::synchronizedImuCallback, this, std::placeholders::_1, std::placeholders::_2));
            // subImuAccelMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuAccelMsgName, 10, std::bind(&MonocularMode::Imu_accel_callback, this, _1));
            // subImuGyroMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuGyroMsgName, 10, std::bind(&MonocularMode::Imu_gyro_callback, this, _1));
        }
    }

    //RGBD topics
    if(sensorType == ORB_SLAM3::System::RGBD || sensorType == ORB_SLAM3::System::IMU_RGBD){
        subDptMsg_ = this->create_subscription<sensor_msgs::msg::Image>(subDptMsgName, 1, std::bind(&MonocularMode::Dpt_callback, this, _1));
        subRGBMsg_ = this->create_subscription<sensor_msgs::msg::Image>(subRGBMsgName, 1, std::bind(&MonocularMode::RGB_callback, this, _1));
        img1_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, subDptMsgName.c_str(), rmw_qos_profile_sensor_data);
        img2_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, subRGBMsgName.c_str(), rmw_qos_profile_sensor_data);
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *img2_sub_filtered_, *img1_sub_filtered_);
        sync_->registerCallback(std::bind(&MonocularMode::synchronizedDepthCallback, this, std::placeholders::_1, std::placeholders::_2));
        if(sensorType == ORB_SLAM3::System::IMU_RGBD){
            // subImuAccMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuAccMsgName, 10, std::bind(&MonocularMode::Imu_accel_callback, this, _1));
            // subImuGyrMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuGyrMsgName, 10, std::bind(&MonocularMode::Imu_gyro_callback, this, _1));
            imu_acc_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuAccMsgName.c_str(), rmw_qos_profile_sensor_data);
            imu_gyr_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuGyrMsgName.c_str(), rmw_qos_profile_sensor_data);
            using SyncPolicy_imu = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>;
            sync_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_imu>>(SyncPolicy_imu(10), *imu_acc_sub_filtered_, *imu_gyr_sub_filtered_);
            sync_imu_->registerCallback(std::bind(&MonocularMode::synchronizedImuCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
    }
    
    //Mono topics

    //common topics
    poseStampedPublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb/pose", 10);

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::IMU_STEREO; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}


void MonocularMode::ImgR_callback(const sensor_msgs::msg::Image& msg){
    std::lock_guard<std::mutex> lock(rimg_mutex_);
    rimg_buff.push_back(msg);
    rimg_cv.notify_one();
}

void MonocularMode::ImgL_callback(const sensor_msgs::msg::Image& msg){
    cv_bridge::CvImagePtr lcv_ptr;
    cv_bridge::CvImagePtr rcv_ptr;
    try{
        lcv_ptr = cv_bridge::toCvCopy(msg);
        img_timestamp = msg.header.stamp;
        this->timeStep = img_timestamp.nanoseconds()/1e9;
        
        if(sensorType==ORB_SLAM3::System::IMU_STEREO){
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it = imu_buff.begin();
            vImuMeas.clear();
            while(it != imu_buff.end()){
                rclcpp::Time tim = it->header.stamp;
                if(tim > last_time_img_ && tim<=img_timestamp){
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
                    it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, tim.nanoseconds()/1e9));
                    it = imu_buff.erase(it);
                }
                else if(tim>img_timestamp){
                    break;
                }
                else{
                    it = imu_buff.erase(it);
                }
            }
        }
        std::unique_lock<std::mutex> lock(rimg_mutex_);
        rimg_cv.wait(lock, [this] { return !rimg_buff.empty(); });
        std::cout<<"buffer size before: "<<rimg_buff.size()<<std::endl;
        auto rit = rimg_buff.begin();
        while(rit != rimg_buff.end()){
            rclcpp::Time tim = rit->header.stamp;
            std::cout<<"buffer size: "<<rimg_buff.size()<<" "<<std::abs(tim.nanoseconds()-img_timestamp.nanoseconds())<<std::endl;
            
            if(std::abs(tim.nanoseconds()-img_timestamp.nanoseconds())<=1000000){
                rcv_ptr = cv_bridge::toCvCopy(*rit);
                rit = rimg_buff.erase(rit);
                break;
            }
            else if(tim.nanoseconds()>(img_timestamp.nanoseconds()+1000000)){
                break;
            }
            else{
                rit = rimg_buff.erase(rit);
            }
        }
    }
    catch (cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    Sophus::SE3f Tcw;
    if(sensorType == ORB_SLAM3::System::STEREO){
        Tcw = pAgent->TrackStereo(lcv_ptr->image, rcv_ptr->image, this->timeStep);
    }
    else{
        Tcw = pAgent->TrackStereo(lcv_ptr->image, rcv_ptr->image, this->timeStep, vImuMeas);
    }
    
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Matrix3f R = Twc.rotationMatrix();
    Eigen::Vector3f t = Twc.translation();

    // Convert to quaternion
    Eigen::Quaternionf q(R);

    // Fill Pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg.header.stamp;  // or use this->get_clock()->now()
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Publish
    poseStampedPublisher_->publish(pose_msg);
    std::cout<<"Pose: "<<t.x()<<t.y()<<t.z()<<std::endl;

    last_time_img_ = img_timestamp;

}

void MonocularMode::Dpt_callback(const sensor_msgs::msg::Image& msg){

}

void MonocularMode::RGB_callback(const sensor_msgs::msg::Image& msg){

}

void MonocularMode::Imu_accel_callback(const sensor_msgs::msg::Imu& msg){
    std::lock_guard<std::mutex> lock(imu_accel_mutex_);
    imu_accel_buff.push_back(msg);
}
void MonocularMode::Imu_gyro_callback(const sensor_msgs::msg::Imu& msg){
    std::lock_guard<std::mutex> lock(imu_gyro_mutex_);
    imu_gyro_buff.push_back(msg);
}

void MonocularMode::synchronizedImuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& acc_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr& gyr_msg
){
    std::lock_guard<std::mutex> lock(imu_mutex_);
    {
        sensor_msgs::msg::Imu msg = *acc_msg;
        msg.angular_velocity.x = gyr_msg->angular_velocity.x;
        msg.angular_velocity.y = gyr_msg->angular_velocity.y;
        msg.angular_velocity.z = gyr_msg->angular_velocity.z;
        imu_buff.push_back(msg);
    }
}

void MonocularMode::synchronizedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& imgl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& imgr_msg
)
{
    cv_bridge::CvImagePtr lcv_ptr;
    cv_bridge::CvImagePtr rcv_ptr;
    try{
        lcv_ptr = cv_bridge::toCvCopy(imgl_msg);
        rcv_ptr = cv_bridge::toCvCopy(imgr_msg);
        img_timestamp = imgl_msg->header.stamp;
        this->timeStep = img_timestamp.nanoseconds()/1e9;
        last_time_img_ = img_timestamp;
        
        if(sensorType==ORB_SLAM3::System::IMU_STEREO){
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it = imu_buff.begin();
            vImuMeas.clear();
            while(it != imu_buff.end()){
                rclcpp::Time tim = it->header.stamp;
                if(tim<=img_timestamp){
                    // std::cout<<"imu : "<<vImuMeas.size()<<std::endl;
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
                    it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, tim.nanoseconds()/1e9));
                    it = imu_buff.erase(it);
                }
                else if(tim>img_timestamp){
                    break;
                }
                else{
                    it = imu_buff.erase(it);
                }
            }
        }
        
    }
    catch (cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    Sophus::SE3f Tcw;
    if(sensorType == ORB_SLAM3::System::STEREO){
        Tcw = pAgent->TrackStereo(lcv_ptr->image, rcv_ptr->image, this->timeStep);
    }
    else{
        Tcw = pAgent->TrackStereo(lcv_ptr->image, rcv_ptr->image, this->timeStep, vImuMeas);
    }
    
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Matrix3f R = Twc.rotationMatrix();
    Eigen::Vector3f t = Twc.translation();

    // Convert to quaternion
    Eigen::Quaternionf q(R);

    // Fill Pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = imgl_msg->header.stamp;  // or use this->get_clock()->now()
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Publish
    poseStampedPublisher_->publish(pose_msg);
    // std::cout<<"Pose: "<<t.x()<<t.y()<<t.z()<<std::endl;

}

void MonocularMode::synchronizedDepthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& imgrgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& imgdpt_msg
)
{
    cv_bridge::CvImagePtr rgbcv_ptr;
    cv_bridge::CvImagePtr dptcv_ptr;
    try{
        rgbcv_ptr = cv_bridge::toCvCopy(imgrgb_msg);
        dptcv_ptr = cv_bridge::toCvCopy(imgdpt_msg);
        img_timestamp = imgrgb_msg->header.stamp;
        this->timeStep = img_timestamp.nanoseconds()/1e9;
        last_time_img_ = img_timestamp;
        
        if(sensorType==ORB_SLAM3::System::IMU_RGBD){
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it = imu_buff.begin();
            vImuMeas.clear();
            while(it != imu_buff.end()){
                rclcpp::Time tim = it->header.stamp;
                if(tim<=img_timestamp){
                    // std::cout<<"imu : "<<vImuMeas.size()<<std::endl;
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
                    it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, tim.nanoseconds()/1e9));
                    it = imu_buff.erase(it);
                }
                else if(tim>img_timestamp){
                    break;
                }
                else{
                    it = imu_buff.erase(it);
                }
            }
        }
        
    }
    catch (cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    Sophus::SE3f Tcw;
    if(sensorType == ORB_SLAM3::System::RGBD){
        Tcw = pAgent->TrackRGBD(rgbcv_ptr->image, dptcv_ptr->image, this->timeStep);
    }
    else{
        Tcw = pAgent->TrackRGBD(rgbcv_ptr->image, dptcv_ptr->image, this->timeStep, vImuMeas);
    }
    
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Matrix3f R = Twc.rotationMatrix();
    Eigen::Vector3f t = Twc.translation();

    // Convert to quaternion
    Eigen::Quaternionf q(R);

    // Fill Pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = imgrgb_msg->header.stamp;  // or use this->get_clock()->now()
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Publish
    poseStampedPublisher_->publish(pose_msg);
    // std::cout<<"Pose: "<<t.x()<<t.y()<<t.z()<<std::endl;

}