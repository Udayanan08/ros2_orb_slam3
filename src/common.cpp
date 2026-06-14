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

    // declare the parameteres
    this->declare_parameter("node_name_arg", "orb_slam_3");
    this->declare_parameter("slam_type", "IMU_RGBD");
    this->declare_parameter("settings_file", "/home/ud/ORB_SLAM3/Examples/Monocular/gazebo_cam.yaml");
    this->declare_parameter("voc_file_arg", "/home/ud/ORB_SLAM3/Vocabulary/ORBvoc.txt.bin");
    this->declare_parameter("cam_topic_name_left", "/camera/image_raw");
    this->declare_parameter("cam_topic_name_right", "/camera/right/image_raw");
    this->declare_parameter("imu_topic_name", "/imu");
    this->declare_parameter("rgb_topic_name", "/rgb/image_raw");
    this->declare_parameter("depth_topic_name", "/depth/image_raw");
    this->declare_parameter("camera_type", "realsense");

    // this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    // this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    // this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    // this->declare_parameter("cam_topic_name_left", "not_set");
    // this->declare_parameter("imu_topic_name", "not_set");
    // this->declare_parameter("dataset_type", "not_set");
    // this->declare_parameter("slam_type", "not_set");

    //* Watchdog, populate default values
    // nodeName = "not_set";
    // vocFilePath = "file_not_set";
    // settingsFilePath = "file_not_set";
    // cam_topic_name_left = "not_set";
    // imu_topic_name = "not_set";
    // slam_type = "not_set";

    //* Populate parameter values

    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath = this->get_parameter("settings_file").as_string();
    cam_topic_name_left = this->get_parameter("cam_topic_name_left").as_string();
    cam_topic_name_right = this->get_parameter("cam_topic_name_right").as_string();
    imu_topic_name = this->get_parameter("imu_topic_name").as_string();
    rgb_topic_name = this->get_parameter("rgb_topic_name").as_string();
    depth_topic_name = this->get_parameter("depth_topic_name").as_string();
    camera_type = this->get_parameter("camera_type").as_string();
    slam_type = this->get_parameter("slam_type").as_string();

    auto qos = rclcpp::SensorDataQoS();


    // std::string pkg_path = ament_index_cpp::get_package_share_directory("ros2_orb_slam3");
    // vocFilePath = pkg_path + "/orb_slam3/Vocabulary/ORBvoc.txt.bin";

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    //subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    if(cam_topic_name_left == "not_set")
    {
        subImgMsgName = "/orb/cam_image";
    }
    else
    {
        subImgMsgName = cam_topic_name_left;
    }
    if(imu_topic_name == "not_set"){
        subImuMsgName = "/imu0";
    }else{
        subImuMsgName = imu_topic_name;
    }
    if(camera_type == "not_set"){ camera_type = "realsense";}
     // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    subImgLMsgName = "/camera/camera/infra1/image_rect_raw";
    subImgRMsgName = "/camera/camera/infra2/image_rect_raw";
    subDptMsgName = "/camera/camera/depth/image_rect_raw";
    subRGBMsgName = "/camera/camera/infra1/image_rect_raw";
    subImuAccMsgName = "/camera/camera/accel/sample";
    subImuGyrMsgName = "/camera/camera/gyro/sample";

    if(settingsFilePath=="file_path_not_set")
    {
        experimentConfig = "/home/ud/IISC/vo/ORB_SLAM3/Examples/Monocular/gazebo_cam";
    }else
    {
        experimentConfig = settingsFilePath;
    }
    // experimentConfig = "/home/ud/IISC/vo/ORB_SLAM3/Examples/RGB-D/RealSense_D435i";
    // experimentConfig = "/home/ud/IISC/vo/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D435i";
    // experimentConfig = "/home/ud/IISC/vo/ORB_SLAM3/Examples/Monocular/gazebo_cam";
    initializeVSLAM(experimentConfig);

    //* subscribe to python node to receive settings
    // expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    // configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    if(sensorType == ORB_SLAM3::System::MONOCULAR || sensorType == ORB_SLAM3::System::IMU_MONOCULAR){
        subImgMsg_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
        if(sensorType == ORB_SLAM3::System::IMU_MONOCULAR){
            if(camera_type == "r"){
                // subImuAccelMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuAccelMsgName, 10, std::bind(&MonocularMode::Imu_accel_callback, this, _1));
                // subImuGyroMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuGyroMsgName, 10, std::bind(&MonocularMode::Imu_gyro_callback, this, _1));
                // imu_acc_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuAccMsgName.c_str(), rmw_qos_profile_sensor_data);
                // imu_gyr_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuGyrMsgName.c_str(), rmw_qos_profile_sensor_data);
                // using SyncPolicy_imu = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>;
                // sync_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_imu>>(SyncPolicy_imu(10), *imu_acc_sub_filtered_, *imu_gyr_sub_filtered_);
                // sync_imu_->registerCallback(std::bind(&MonocularMode::synchronizedImuCallback, this, std::placeholders::_1, std::placeholders::_2));
            }
            else if (camera_type == "reealsense")
            {
                subImuMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuMsgName, 100, std::bind(&MonocularMode::ImuCallback, this, _1));
            }    
        }
    }

    //* subscribe to receive the timestep
    // subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //Stereo topics
    if(sensorType == ORB_SLAM3::System::STEREO || sensorType == ORB_SLAM3::System::IMU_STEREO){
        img1_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, cam_topic_name_left.c_str(), rmw_qos_profile_sensor_data);
        img2_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, cam_topic_name_right.c_str(), rmw_qos_profile_sensor_data);
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *img1_sub_filtered_, *img2_sub_filtered_);
        sync_->registerCallback(std::bind(&MonocularMode::synchronizedCallback, this, std::placeholders::_1, std::placeholders::_2));
        // subImgRMsg_ = this->create_subscription<sensor_msgs::msg::Image>(subImgRMsgName, 10, std::bind(&MonocularMode::ImgR_callback, this, _1));
        // subImgLMsg_ = this->create_subscription<sensor_msgs::msg::Image>(subImgLMsgName, 1, std::bind(&MonocularMode::ImgL_callback, this, _1));
        if(sensorType == ORB_SLAM3::System::IMU_STEREO){
            if(camera_type == "realsense"){
                subImuMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuMsgName, 100, std::bind(&MonocularMode::ImuCallback, this, _1));
            }
            // imu_acc_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuAccMsgName.c_str(), rmw_qos_profile_sensor_data);
            // imu_gyr_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuGyrMsgName.c_str(), rmw_qos_profile_sensor_data);
            // using SyncPolicy_imu = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>;
            // sync_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_imu>>(SyncPolicy_imu(10), *imu_acc_sub_filtered_, *imu_gyr_sub_filtered_);
            // sync_imu_->registerCallback(std::bind(&MonocularMode::synchronizedImuCallback, this, std::placeholders::_1, std::placeholders::_2));
            // subImuAccelMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuAccelMsgName, 10, std::bind(&MonocularMode::Imu_accel_callback, this, _1));
            // subImuGyroMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuGyroMsgName, 10, std::bind(&MonocularMode::Imu_gyro_callback, this, _1));
        }
    }

    //RGBD topics
    if(sensorType == ORB_SLAM3::System::RGBD || sensorType == ORB_SLAM3::System::IMU_RGBD){
        // subDptMsg_ = this->create_subscription<sensor_msgs::msg::Image>(depth_topic_name, 1, std::bind(&MonocularMode::Dpt_callback, this, _1));
        // subRGBMsg_ = this->create_subscription<sensor_msgs::msg::Image>(rgb_topic_name, 1, std::bind(&MonocularMode::RGB_callback, this, _1));
        img1_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic_name.c_str(), rmw_qos_profile_sensor_data);
        img2_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rgb_topic_name.c_str(), rmw_qos_profile_sensor_data);
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *img2_sub_filtered_, *img1_sub_filtered_);
        sync_->registerCallback(std::bind(&MonocularMode::synchronizedDepthCallback, this, std::placeholders::_1, std::placeholders::_2));
        if(sensorType == ORB_SLAM3::System::IMU_RGBD){
            if(camera_type == "realsense"){
                subImuMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, qos, std::bind(&MonocularMode::ImuCallback, this, _1));
            }
            // subImuAccMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuAccMsgName, 10, std::bind(&MonocularMode::Imu_accel_callback, this, _1));
            // subImuGyrMsg_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuGyrMsgName, 10, std::bind(&MonocularMode::Imu_gyro_callback, this, _1));
            // imu_acc_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuAccMsgName.c_str(), rmw_qos_profile_sensor_data);
            // imu_gyr_sub_filtered_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, subImuGyrMsgName.c_str(), rmw_qos_profile_sensor_data);
            // using SyncPolicy_imu = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>;
            // sync_imu_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_imu>>(SyncPolicy_imu(10), *imu_acc_sub_filtered_, *imu_gyr_sub_filtered_);
            // sync_imu_->registerCallback(std::bind(&MonocularMode::synchronizedImuCallback, this, std::placeholders::_1, std::placeholders::_2));
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
    
    // settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(""); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    if(slam_type == "not_set" || slam_type == "MONOCULAR"){
        sensorType = ORB_SLAM3::System::MONOCULAR;
    }
    else if(slam_type == "IMU_MONOCULAR"){
        sensorType = ORB_SLAM3::System::IMU_MONOCULAR;
    }
    else if(slam_type == "IMU_STEREO"){
        sensorType = ORB_SLAM3::System::IMU_STEREO;
    }
    else if(slam_type == "STEREO"){
        sensorType = ORB_SLAM3::System::STEREO;
    }
    else if(slam_type == "IMU_RGBD"){
        sensorType = ORB_SLAM3::System::IMU_RGBD;
    }
    else if(slam_type == "RGBD"){
        sensorType = ORB_SLAM3::System::RGBD;
    }
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << slam_type << " node initialized" << std::endl; // TODO needs a better message
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
    cv::Mat img_cv8uc1;
    std::vector<ORB_SLAM3::IMU::Point> localImuMeas;
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
            cv_ptr = cv_bridge::toCvCopy(msg);
            img_timestamp = msg.header.stamp;
            cv::Mat img_mono_16 = cv_ptr->image;
            img_mono_16.convertTo(img_cv8uc1, CV_8UC1, 1.0/256.0);
            this->timeStep = img_timestamp.nanoseconds() / 1e9;

            // if(sensorType == ORB_SLAM3::System::IMU_MONOCULAR){
            //     // std::lock_guard<std::mutex> lock(imu_mutex_);
            //     std::vector<sensor_msgs::msg::Imu> imu_buff_copy;

            //     {
            //         std::lock_guard<std::mutex> lock(imu_mutex_);
            //         imu_buff_copy = imu_buff;
            //         imu_buff.clear();  // clear original — IMU callback can now push freely
            //     } 
                
            //     int64_t img_tim = img_timestamp.nanoseconds();
            //     const int64_t IMU_CAM_OFFSET_NS = 25000000LL;  // 25ms
            //     const int64_t MAX_WINDOW_NS     = 30000000LL;  // 30ms max — never exceed one frame width

            //     // ✅ Clamp the lower bound so window never exceeds MAX_WINDOW_NS
            //     int64_t effective_last = std::max(last_time_img_, img_tim - MAX_WINDOW_NS);

            //     auto it = imu_buff_copy.begin();
            //     localImuMeas.clear();

            //     while(it != imu_buff_copy.end()){
            //         rclcpp::Time imu_time(it->header.stamp.sec, it->header.stamp.nanosec, RCL_ROS_TIME);
            //         int64_t tim = imu_time.nanoseconds();

            //         // std::cout << "tim: " << tim 
            //         //         << " last: " << effective_last
            //         //         << " img: "  << img_tim << std::endl;

            //         if(tim > effective_last && tim <= img_tim + IMU_CAM_OFFSET_NS){
            //             localImuMeas.push_back(ORB_SLAM3::IMU::Point(
            //                 it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
            //                 it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z,
            //                 tim / 1e9));
            //             it = imu_buff_copy.erase(it);
            //         }
            //         else if(tim > img_tim + IMU_CAM_OFFSET_NS){
            //             break;
            //         }
            //         else{
            //             it = imu_buff_copy.erase(it);  // discard anything older than the clamped window
            //         }
            //     }

            //     std::cout << "Collected IMU: " << localImuMeas.size() << std::endl;

            //     if(localImuMeas.size() < 1){
            //         RCLCPP_WARN(this->get_logger(), 
            //             "Too few IMU measurements (%zu), skipping", localImuMeas.size());
            //         return;  // don't update last_time_img_
            //     }
            // }

            // // ✅ only update on successful frames
            // last_time_img_ = img_timestamp.nanoseconds();

            if(sensorType == ORB_SLAM3::System::IMU_MONOCULAR){

                std::vector<sensor_msgs::msg::Imu> imu_buff_copy;
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    imu_buff_copy = imu_buff;
                    imu_buff.clear();
                }

                int64_t img_tim = img_timestamp.nanoseconds();
                const int64_t IMU_CAM_OFFSET_NS = 40000000LL;  // 40ms
                const int64_t MAX_WINDOW_NS     = 35000000LL;  // ~1.5 frames max
                const size_t  MIN_IMU_COUNT     = 3;           // minimum to proceed

                // Clamp window to avoid collecting too many after a skip
                int64_t effective_last = std::max(last_time_img_, img_tim - MAX_WINDOW_NS);

                localImuMeas.clear();
                std::vector<sensor_msgs::msg::Imu> future_msgs;

                for(size_t i = 0; i < imu_buff_copy.size(); i++){
                    auto& imu_msg = imu_buff_copy[i];
                    rclcpp::Time imu_time(imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec, RCL_ROS_TIME);
                    int64_t tim = imu_time.nanoseconds();

                    if(tim > effective_last && tim <= img_tim + IMU_CAM_OFFSET_NS){
                        localImuMeas.push_back(ORB_SLAM3::IMU::Point(
                            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                            tim / 1e9));
                    }
                    else if(tim > img_tim + IMU_CAM_OFFSET_NS){
                        // save remaining for next frame
                        future_msgs.insert(future_msgs.end(),
                            imu_buff_copy.begin() + i,
                            imu_buff_copy.end());
                        break;
                    }
                    // old messages discarded
                }

                // Put future messages back
                if(!future_msgs.empty()){
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    imu_buff.insert(imu_buff.begin(), future_msgs.begin(), future_msgs.end());
                }

                std::cout << "Collected IMU: " << localImuMeas.size() << std::endl;

                if(localImuMeas.size() < MIN_IMU_COUNT){
                    RCLCPP_WARN(this->get_logger(),
                        "Too few IMU measurements (%zu), skipping", localImuMeas.size());
                    // ✅ do NOT update last_time_img_ — let next frame recollect
                    return;
                }
            }

            last_time_img_ = img_timestamp.nanoseconds(); // ✅ only on success
        


        
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
    Sophus::SE3f Tcw;
    if(sensorType == ORB_SLAM3::System::STEREO){
        Tcw = pAgent->TrackMonocular(img_cv8uc1, this->timeStep);
    }
    else if(sensorType == ORB_SLAM3::System::MONOCULAR){
        Tcw = pAgent->TrackMonocular(cv_ptr->image, this->timeStep);
        std::cout<<"img sent"<<std::endl;
    }
    else if(sensorType == ORB_SLAM3::System::RGBD){
        // Tcw = pAgent->TrackM
    }
    else{
        Tcw = pAgent->TrackMonocular(cv_ptr->image, this->timeStep, vImuMeas);
        std::cout<<"img and imu sent"<<std::endl;
    }
    // Sophus::SE3f Tcw = pAgent->TrackMonocular(img_cv8uc1, timeStep); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

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

    last_time_img_ = img_timestamp.nanoseconds();

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
                int64_t tim = it->header.stamp.nanosec;
                if(tim > last_time_img_ && tim<=img_timestamp.nanoseconds()){
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
                    it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, tim/1e9));
                    it = imu_buff.erase(it);
                }
                else if(tim>img_timestamp.nanoseconds()){
                    break;
                }
                else{
                    it = imu_buff.erase(it);
                }
            }
        }
        // std::unique_lock<std::mutex> lock(rimg_mutex_);
        // rimg_cv.wait(lock, [this] { return !rimg_buff.empty(); });
        // std::cout<<"buffer size before: "<<rimg_buff.size()<<std::endl;
        // auto rit = rimg_buff.begin();
        // while(rit != rimg_buff.end()){
        //     int64_t tim = rit->header.stamp.nanosec;
        //     std::cout<<"buffer size: "<<rimg_buff.size()<<" "<<std::abs(tim-img_timestamp.nanoseconds())<<std::endl;
            
        //     if(std::abs(tim-img_timestamp.nanoseconds())<=1000000){
        //         rcv_ptr = cv_bridge::toCvCopy(*rit);
        //         rit = rimg_buff.erase(rit);
        //         break;
        //     }
        //     else if(tim>(img_timestamp.nanoseconds()+1000000)){
        //         break;
        //     }
        //     else{
        //         rit = rimg_buff.erase(rit);
        //     }
        // }
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

    last_time_img_ = img_timestamp.nanoseconds();

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

void MonocularMode::ImuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg
){
    std::lock_guard<std::mutex> lock(imu_mutex_);
    {
        sensor_msgs::msg::Imu msg = *imu_msg;
        imu_buff.push_back(*imu_msg);
        // std::cout<<"message :"<<imu_buff.size()<<std::endl;
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
        last_time_img_ = img_timestamp.nanoseconds();
        
        if(sensorType==ORB_SLAM3::System::IMU_STEREO){
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it = imu_buff.begin();
            vImuMeas.clear();
            while(it != imu_buff.end()){
                int64_t tim = it->header.stamp.nanosec;
                if(tim<=img_timestamp.nanoseconds()){
                    // std::cout<<"imu : "<<vImuMeas.size()<<std::endl;
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
                    it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, tim/1e9));
                    it = imu_buff.erase(it);
                }
                else if(tim>img_timestamp.nanoseconds()){
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
    const sensor_msgs::msg::Image::ConstSharedPtr &imgrgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &imgdpt_msg
)
{
    cv_bridge::CvImagePtr rgbcv_ptr;
    cv_bridge::CvImagePtr dptcv_ptr;
    // std::cout << "message received !"<<std::endl;
    
    try{
        rgbcv_ptr = cv_bridge::toCvCopy(imgrgb_msg);
        dptcv_ptr = cv_bridge::toCvCopy(imgdpt_msg);
        // std::cout << "RGB type: "
        //   << rgbcv_ptr->image.type()
        //   << " channels: "
        //   << rgbcv_ptr->image.channels()
        //   << std::endl;
        img_timestamp = imgrgb_msg->header.stamp;
        this->timeStep = img_timestamp.nanoseconds()/1e9;
        last_time_img_ = img_timestamp.nanoseconds();
        
        if(sensorType==ORB_SLAM3::System::IMU_RGBD){
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it = imu_buff.begin();
            vImuMeas.clear();
            while(it != imu_buff.end()){
                int64_t tim = it->header.stamp.nanosec;
                if(tim<=img_timestamp.nanoseconds()){
                    // std::cout<<"imu : "<<vImuMeas.size()<<std::endl;
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
                    it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, tim/1e9));
                    it = imu_buff.erase(it);
                }
                else if(tim>img_timestamp.nanoseconds()){
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
