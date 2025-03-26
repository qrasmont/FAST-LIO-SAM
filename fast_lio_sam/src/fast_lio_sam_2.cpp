#include "fast_lio_sam/fast_lio_sam_2.h"

using namespace std::placeholders;
using namespace std::chrono_literals;
// using namespace fs = std::filesystem;
bool DEBUG = false;

FastLioSam::FastLioSam() : Node("fast_lio_sam_node")
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    loadParams();

    initPublishers();

    initSubscribers();

    initTimers();
    pose_update_count_ = 0;
    loop_closure_.reset(new LoopClosure(lc_config_));

    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;

    RCLCPP_INFO(this->get_logger(), "Main class, starting node..");

    geometry_msgs::msg::PoseStamped fake_pose;
    savePoseToYaml(std::make_shared<geometry_msgs::msg::PoseStamped>(fake_pose), yaml_file_name_);
    savePoseToYaml(std::make_shared<geometry_msgs::msg::PoseStamped>(fake_pose), yaml_file_name_bkp_);
}

FastLioSam::~FastLioSam()
{
    // // save map
    // if (save_map_bag_)
    // {
    //     rosbag::Bag bag;
    //     bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
    //     {
    //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
    //         for (size_t i = 0; i < keyframes_.size(); ++i)
    //         {
    //             ros::Time time;
    //             time.fromSec(keyframes_[i].timestamp_);
    //             bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
    //             bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
    //         }
    //     }
    //     bag.close();
    //     ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    // }
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd", *voxelized_map);
        RCLCPP_INFO(this->get_logger(), "\033[32;1mResult saved in .pcd format!!!\033[0m");
    }
}


void FastLioSam::loadParams()
{
    this->declare_parameter("basic.map_frame", "map");
    this->declare_parameter("basic.loop_update_hz", 1.0);
    this->declare_parameter("basic.vis_hz", 0.5);

    this->declare_parameter("keyframe.keyframe_threshold", 1.0);
    this->declare_parameter("keyframe.num_submap_keyframes", 5);
    
    this->declare_parameter("loop.loop_detection_radius", 15.0);
    this->declare_parameter("loop.loop_detection_timediff_threshold", 10.0);
    
    this->declare_parameter("icp.icp_voxel_resolution", 0.3);
    this->declare_parameter("icp.icp_score_threshold", 0.3);

    this->declare_parameter("result.save_voxel_resolution", 0.3);
    this->declare_parameter("result.save_map_pcd", false);
    this->declare_parameter("result.save_map_bag", false);
    this->declare_parameter("result.save_in_kitti_format", false);
    this->declare_parameter("result.seq_name", "");
    this->declare_parameter("result.save_pose_yml", false);
    this->declare_parameter("result.yaml_file_name", "");
    this->declare_parameter("result.yaml_file_name_bkp", "");
    this->declare_parameter("result.bkp_dt", 1);
    this->declare_parameter("result.map_publish_freq", 10);


    this->get_parameter("basic.map_frame", map_frame_);
    this->get_parameter("basic.loop_update_hz", loop_update_hz_);
    this->get_parameter("basic.vis_hz", vis_hz_);

    this->get_parameter("keyframe.keyframe_threshold", keyframe_thr_);
    this->get_parameter("keyframe.num_submap_keyframes", lc_config_.num_submap_keyframes_);
    
    this->get_parameter("loop.loop_detection_radius", lc_config_.loop_detection_radius_);
    this->get_parameter("loop.loop_detection_timediff_threshold", lc_config_.loop_detection_timediff_threshold_);
    lc_config_.icp_max_corr_dist_ = lc_config_.loop_detection_radius_ * 1.5;
    
    this->get_parameter("icp.icp_voxel_resolution", lc_config_.voxel_res_);
    this->get_parameter("icp.icp_score_threshold", lc_config_.icp_score_threshold_);
    
    this->get_parameter("result.save_voxel_resolution", voxel_res_);
    this->get_parameter("result.save_map_pcd", save_map_pcd_);
    this->get_parameter("result.save_map_bag", save_map_bag_);
    this->get_parameter("result.save_in_kitti_format", save_in_kitti_format_);
    this->get_parameter("result.seq_name", seq_name_);
    this->get_parameter("result.save_pose_yml", save_pose_yml_);
    this->get_parameter("result.yaml_file_name", yaml_file_name_);
    this->get_parameter("result.yaml_file_name_bkp", yaml_file_name_bkp_);
    this->get_parameter("result.bkp_dt", bkp_dt_);
    this->get_parameter("result.map_publish_freq", map_publish_freq_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void FastLioSam::initPublishers()
{
    odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ori_odom", 10);
    corrected_odom_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_odom", 10);
    corrected_pcd_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_map", 10);
    corrected_current_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_current_pcd", 10);
    debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("src", 10);
    debug_dst_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dst", 10);
    debug_fine_aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned", 10);
    realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("ori_path", 10);
    corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("corrected_path", 10);
    loop_detection_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("loop_detection", 10);
    
}

void FastLioSam::initSubscribers()
{
    odom_sub_ = std::make_unique<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "Odometry");
    pcd_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cloud_registered");
    sub_odom_pcd_sync_ = std::make_unique<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10),*odom_sub_, *pcd_sub_);
    sub_odom_pcd_sync_->registerCallback(std::bind(&FastLioSam::odomPcdCallback, this, _1, _2));

    sub_save_flag_ = this->create_subscription<std_msgs::msg::String>("save_dir", 1, std::bind(&FastLioSam::saveFlagCallback, this, _1));
}

void FastLioSam::initTimers()
{
    loop_timer_ = this->create_wall_timer(500ms, std::bind(&FastLioSam::loopTimerCallback, this));
    vis_timer_ = this->create_wall_timer(500ms, std::bind(&FastLioSam::visTimerCallback, this));
}

geometry_msgs::msg::TransformStamped FastLioSam::getTransformStamped(const tf2::Transform &transform, const std::string &frame_id, const std::string &child_frame_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Populate the TransformStamped message
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;

    transform_stamped.transform.translation.x = transform.getOrigin().x();
    transform_stamped.transform.translation.y = transform.getOrigin().y();
    transform_stamped.transform.translation.z = transform.getOrigin().z();

    tf2::Quaternion quat = transform.getRotation();
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();

    return transform_stamped;

}

void FastLioSam::savePoseToYaml(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg, const std::string& filename){
    try{
        YAML::Emitter yaml_emitter;
        yaml_emitter << YAML::BeginMap;
        
            yaml_emitter << YAML::Key << "header";
            yaml_emitter << YAML::Value << YAML::BeginMap;
                yaml_emitter << YAML::Key << "frame_id";
                yaml_emitter << YAML::Value << pose_msg->header.frame_id;
                yaml_emitter << YAML::Key << "timestamp";
                std::stringstream ss;
                ss << pose_msg->header.stamp.sec << "." << std::setw(9) << std::setfill('0') << pose_msg->header.stamp.nanosec;
                std::string timestamp = ss.str();
                yaml_emitter << YAML::Value << timestamp;
            yaml_emitter << YAML::EndMap;
            yaml_emitter << YAML::Key << "pose";
            yaml_emitter << YAML::Value << YAML::BeginMap;
                yaml_emitter << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
                    yaml_emitter << YAML::Key << "x" << YAML::Value << pose_msg->pose.position.x;
                    yaml_emitter << YAML::Key << "y" << YAML::Value << pose_msg->pose.position.y;
                    yaml_emitter << YAML::Key << "z" << YAML::Value << pose_msg->pose.position.z;
                yaml_emitter << YAML::EndMap;
                yaml_emitter << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
                    yaml_emitter << YAML::Key << "x" << YAML::Value << pose_msg->pose.orientation.x;
                    yaml_emitter << YAML::Key << "y" << YAML::Value << pose_msg->pose.orientation.y;
                    yaml_emitter << YAML::Key << "z" << YAML::Value << pose_msg->pose.orientation.z;
                    yaml_emitter << YAML::Key << "w" << YAML::Value << pose_msg->pose.orientation.w;
                yaml_emitter << YAML::EndMap; 
            yaml_emitter << YAML::EndMap;

        yaml_emitter << YAML::EndMap;

        // // Get the current directory
        // std::filesystem::path current_path = std::filesystem::current_path();

        // // Move one directory up
        // std::filesystem::path parent_path = current_path.parent_path();

        // // Enter the "config" directory
        std::filesystem::path config_path = "/ros2_dep/install/fast_lio_sam/share/fast_lio_sam/config";
        
        // Ensure the directory exists
        if (!std::filesystem::exists(config_path)) {
            RCLCPP_ERROR(this->get_logger(), "Error: Config directory does not exist: %s", config_path.string().c_str());
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Config directory found at: %s", config_path.string().c_str());
        // RCLCPP_INFO(this->get_logger(), "YAML file name: %s", filename.c_str());
        std::filesystem::path yaml_absolute_path = config_path/filename;
        // std::filesystem::path yaml_absolute_path = /root/ros2_ws/src/
        std::ofstream fout(yaml_absolute_path);
        fout << yaml_emitter.c_str();
        fout.close();
        // std::cout << "YAML file saved to: " << yaml_absolute_path.c_str() << std::endl;
    } catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        return;
    }
    
}

void FastLioSam::odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg)
{   
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odomcb 1"); }
    
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_);
    auto t1 = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped transform_stamped;
    tf2::Transform transform;
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 2"); }

    {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
        if (save_pose_yml_) { 
            savePoseToYaml(std::make_shared<geometry_msgs::msg::PoseStamped>(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_)), yaml_file_name_); 
            if(pose_update_count_ % bkp_dt_ == 0){
                savePoseToYaml(std::make_shared<geometry_msgs::msg::PoseStamped>(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_)), yaml_file_name_bkp_);
            }
            pose_update_count_++;

        }
        realtime_pose_pub_->publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        // broadcaster
        transform = poseEigToROSTf2(current_frame_.pose_corrected_eig_);
        transform_stamped = getTransformStamped(transform, map_frame_, "robot");
        tf_broadcaster_->sendTransform(transform_stamped);
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 3"); }
    }
    corrected_current_pcd_pub_->publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    if (!is_initialized_)
    {
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 4"); }
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished(); // rad*rad,
                                                                                                    // meter*meter
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
        init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        is_initialized_ = true;
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 5");}
    }
    else
    {
        //// 2. check if keyframe
        auto t2 = this->get_clock()->now();
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 6"); }
        if (checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 7"); }
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
                if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 8"); }
            }
            // 2-3. if so, add to graph
            auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
            gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
            gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
            if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 9"); }
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                                    current_keyframe_idx_,
                                                                    pose_from.between(pose_to),
                                                                    odom_noise));
                init_esti_.insert(current_keyframe_idx_, pose_to);
            }
            current_keyframe_idx_++;
            if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 10");}

            //// 3. vis
            auto t3 = this->get_clock()->now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
                if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 11"); }
            }

            //// 4. optimize with graph
            auto t4 = this->get_clock()->now();
            // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
            {
                if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 12"); }
                std::lock_guard<std::mutex> lock(graph_mutex_);
                isam_handler_->update(gtsam_graph_, init_esti_);
                isam_handler_->update();
                if (loop_added_flag_) // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
                {
                    isam_handler_->update();
                    isam_handler_->update();
                    isam_handler_->update();
                }
                gtsam_graph_.resize(0);
                init_esti_.clear();
            }

            //// 5. handle corrected results
            // get corrected poses and reset odom delta (for realtime pose pub)
            auto t5 = this->get_clock()->now();
            {
                if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 13"); }
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                corrected_esti_ = isam_handler_->calculateEstimate();
                last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
            // correct poses in keyframes
            if (loop_added_flag_)
            {
                if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 14"); }
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for (size_t i = 0; i < corrected_esti_.size(); ++i)
                {
                    keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                }
                loop_added_flag_ = false;
                if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "odom cb 15"); }
            }
            auto t6 = this->get_clock()->now();

            auto diff_2_1 = t2 - t1;
            auto diff_3_2 = t3 - t2;
            auto diff_4_3 = t4 - t3;
            auto diff_5_4 = t5 - t4;
            auto diff_6_5 = t6 - t5;
            auto diff_6_1 = t6 - t1;

            RCLCPP_INFO(this->get_logger(), "real: %f, key_add: %f, vis: %f, opt: %f, res: %f, tot: %f",
                    diff_2_1.seconds(),
                    diff_3_2.seconds(),
                    diff_4_3.seconds(),
                    diff_5_4.seconds(),
                    diff_6_5.seconds(),
                    diff_6_1.seconds());
        }
    }
    return;
}

void FastLioSam::loopTimerCallback()
{
    loop_closure_.reset(new LoopClosure(lc_config_));
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "loop timer 1"); }
    auto &latest_keyframe = keyframes_.back();
    if (!is_initialized_ || keyframes_.empty() || latest_keyframe.processed_) { return; }
    latest_keyframe.processed_ = true;
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "loop timer 2"); }

    auto t1 = this->get_clock()->now();
    const int closest_keyframe_idx = loop_closure_->fetchClosestKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "loop timer 3"); }
    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mLoop closure accepted. Score: %.3f\033[0m", reg_output.score_); 
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "after acceptance"); }
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "graph mutex"); }
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx}); // for vis
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "all bools true"); }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Loop closure rejected. Score: %f", reg_output.score_);
    }
    auto t2 = this->get_clock()->now();


    debug_src_pub_->publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));

    RCLCPP_INFO(this->get_logger(), "loop: %f", (t2 - t1).seconds());
    return;
}

void FastLioSam::visTimerCallback()
{
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "vis timer 1"); }
    if (!is_initialized_)
    {
        return;
    }
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "vis_timer 2"); }

    auto tv1 = this->get_clock()->now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::msg::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty())
        {
            loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {
            if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "vis timer before corrected data"); }
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
            if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "vis timer after corrected data"); }
        }
        loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "vis timer for publishing"); }
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
        path_pub_->publish(odom_path_);
        corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_->publish(corrected_path_);
        vis_count_++;
        if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "vis timer published"); }
    }

    //// 3. global map
    if (global_map_vis_switch_) // save time, only once in num_keyframes_per_map_publish keyframes
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && ((vis_count_ - 1) % map_publish_freq_ == 0))
    {
        global_map_vis_switch_ = true;
    }
    auto tv2 = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "vis: %f", (tv2 - tv1).seconds());
    return;
}

void FastLioSam::saveFlagCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string save_dir = msg->data != "" ? msg->data : package_path_;

    // save scans as individual pcd files and poses in KITTI format
    // Delete the scans folder if it exists and create a new one
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_)
    {
        RCLCPP_INFO(this->get_logger(), "\033[32;1mScans are saved in %s, following the KITTI and TUM format\033[0m", scans_directory.c_str());
        // if (std::filesystemexists(seq_directory))
        // {
        //     std::filesystemremove_all(seq_directory);
        // }
        // std::filesystemcreate_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
        std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                // Save the point cloud
                std::stringstream ss_;
                ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
                pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

                // Save the pose in KITTI format
                const auto &pose_ = keyframes_[i].pose_corrected_eig_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
                tum_pose_file << std::fixed << std::setprecision(8) << keyframes_[i].timestamp_
                              << " " << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        RCLCPP_INFO(this->get_logger(), "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }

    // if (save_map_bag_)
    // {
    //     rosbag::Bag bag;
    //     bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
    //     {
    //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
    //         for (size_t i = 0; i < keyframes_.size(); ++i)
    //         {
    //             ros::Time time;
    //             time.fromSec(keyframes_[i].timestamp_);
    //             bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
    //             bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
    //         }
    //     }
    //     bag.close();
    //     ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    // }

    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
        RCLCPP_INFO(this->get_logger(), "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
    }
}

visualization_msgs::msg::Marker FastLioSam::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::msg::Marker edges;
    edges.type = 5u;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i)
    {
        if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
            loop_idx_pairs_[i].second >= corrected_esti_in.size())
        {
            continue;
        }
        gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
        gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
        geometry_msgs::msg::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

void FastLioSam::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

bool FastLioSam::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "checkifkeyframe"); }
    bool result = keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
    if ( DEBUG ) { RCLCPP_INFO(this->get_logger(), "result: %d", result); }
    return result;
}
