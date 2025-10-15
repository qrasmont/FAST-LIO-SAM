#include "fast_lio_sam/fast_lio_sam_2.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

using namespace std::placeholders;
using namespace std::chrono;

bool DEBUG = false;

std::string tf_topic = "/tf";
std::string tf_static_topic = "/tf_static";

FastLioSam::FastLioSam() : Node("fast_lio_sam_node")
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    loadParams();

    loop_closure_.reset(new LoopClosure(lc_config_));

    if (!bag_file_.empty())
    {
        // Offline mode
        RCLCPP_INFO(this->get_logger(), "Bag file provided [%s], running in Offline mode.", bag_file_.c_str());
        initPublishers();
        std::thread offline_thread(&FastLioSam::runOffline, this);
        offline_thread.detach();
    }
    else
    {
        // Online mode
        RCLCPP_INFO(this->get_logger(), "No bag file provided, running in Online mode.");
        initPublishers();
        initSubscribers();
        initTimers();
    }

    pose_update_count_ = 0;

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

void FastLioSam::performLoopClosureForKf(size_t keyframe_idx)
{
    auto& keyframe = keyframes_[keyframe_idx];
    if (keyframe.processed_) return;
    keyframe.processed_ = true;

    const int closest_keyframe_idx = loop_closure_->fetchClosestKeyframeIdx(keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }

    const RegistrationOutput& reg_output = loop_closure_->performLoopClosure(keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mLoop closure found between kframe %zu and %d. Score: %.3f\033[0m", keyframe_idx, closest_keyframe_idx, reg_output.score_);
        const auto& score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * keyframe.pose_corrected_eig_);
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({keyframe.idx_, closest_keyframe_idx});
        loop_added_flag_ = true; // Signal that a loop has been added
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;31mLoop closure between kframe %zu and %d rejected. Score: %.3f\033[0m", keyframe_idx, closest_keyframe_idx, reg_output.score_);
    }
}


void FastLioSam::runOffline()
{
    RCLCPP_INFO(this->get_logger(), "Starting offline processing from bag: %s", bag_file_.c_str());
    rclcpp::Rate rate(200.0);

    FastLioConfig config;
    std::string fast_lio_pkg_path;
    try {
        fast_lio_pkg_path = ament_index_cpp::get_package_share_directory("fast_lio");
    } catch (const ament_index_cpp::PackageNotFoundError& e) {
        RCLCPP_ERROR(this->get_logger(), "fast_lio package not found: %s", e.what());
        return;
    }

    std::string config_file_path = fast_lio_pkg_path + "/config/" + fast_lio_config_;
    YAML::Node config_yaml;
    try {
        config_yaml = YAML::LoadFile(config_file_path);
    } catch (const YAML::BadFile & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load fast_lio config file: %s", config_file_path.c_str());
        return;
    }

    YAML::Node params = config_yaml.begin()->second["ros__parameters"];
    if (!params) {
        RCLCPP_ERROR(this->get_logger(), "Could not find 'ros__parameters' in %s", config_file_path.c_str());
        return;
    }

    try {
        config.point_filter_num = params["point_filter_num"].as<int>();
        config.max_iteration = params["max_iteration"].as<int>();
        config.filter_size_surf = params["filter_size_surf"].as<double>();
        config.filter_size_map = params["filter_size_map"].as<double>();
        config.cube_side_length = params["cube_side_length"].as<double>();
        config.runtime_pos_log_enable = params["runtime_pos_log_enable"].as<bool>();
        YAML::Node common_params = params["common"];
        config.time_sync_en = common_params["time_sync_en"].as<bool>();
        config.time_offset_lidar_to_imu = common_params["time_offset_lidar_to_imu"].as<double>();
        YAML::Node preprocess_params = params["preprocess"];
        config.lidar_type = preprocess_params["lidar_type"].as<int>();
        config.scan_line = preprocess_params["scan_line"].as<int>();
        config.blind = preprocess_params["blind"].as<double>();
        config.timestamp_unit = preprocess_params["timestamp_unit"].as<int>();
        config.scan_rate = preprocess_params["scan_rate"].as<int>();
        YAML::Node mapping_params = params["mapping"];
        config.acc_cov = mapping_params["acc_cov"].as<double>();
        config.gyr_cov = mapping_params["gyr_cov"].as<double>();
        config.b_acc_cov = mapping_params["b_acc_cov"].as<double>();
        config.b_gyr_cov = mapping_params["b_gyr_cov"].as<double>();
        config.fov_degree = mapping_params["fov_degree"].as<double>();
        config.det_range = mapping_params["det_range"].as<double>();
        config.extrinsic_est_en = mapping_params["extrinsic_est_en"].as<bool>();
        config.extrinsic_T = mapping_params["extrinsic_T"].as<std::vector<double>>();
        config.extrinsic_R = mapping_params["extrinsic_R"].as<std::vector<double>>();
        config.pcd_save_en = false;
        config.log_path = "/tmp/";
        config.dense_publish_en = false;
        config.map_pub_en = true;
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error while parsing YAML file: %s", e.what());
        return;
    }
    fast_lio_core_ = std::make_unique<FastLioCore>(config);

    std::string storage_id = "";

    rosbag2_storage::StorageOptions storage_options({bag_file_, storage_id});
    rosbag2_cpp::ConverterOptions converter_options;
    rosbag2_cpp::readers::SequentialReader reader;
    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open bag file: %s", e.what());
        return;
    }

    std::string lid_topic = params["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = params["common"]["imu_topic"].as<std::string>();
    rosbag2_storage::StorageFilter filter;
    filter.topics = {lid_topic, imu_topic, tf_topic, tf_static_topic};
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serialization;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

    if (offline_buffered_read_)
    {
        RCLCPP_INFO(this->get_logger(), "Offline mode with BUFFERED reading.");

        RCLCPP_INFO(this->get_logger(), "Reading initial messages for IMU initialization...");
        deque<sensor_msgs::msg::Imu::ConstSharedPtr> init_imu_data;
        while (reader.has_next() && init_imu_data.size() < INIT_IMU_COUNT) {
            auto serialized_msg = reader.read_next();
            if (serialized_msg->topic_name == imu_topic) {
                auto msg = std::make_shared<sensor_msgs::msg::Imu>();
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);
                imu_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                init_imu_data.push_back(msg);
            }
        }

        if (init_imu_data.size() < INIT_IMU_COUNT) {
            RCLCPP_ERROR(this->get_logger(), "Not enough IMU messages in bag to initialize. Found %zu, need %d.", init_imu_data.size(), INIT_IMU_COUNT);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);
            fast_lio_core_->imu_buffer_ = init_imu_data;
        }
        fast_lio_core_->initial_setup();
        RCLCPP_INFO(this->get_logger(), "IMU Initialized.");

        reader.seek(0);
        std::deque<StampedMessage> message_buffer;

        while(reader.has_next() && rclcpp::ok()) {
            while(reader.has_next()) {
                if (!message_buffer.empty() && 
                    (message_buffer.back().timestamp - message_buffer.front().timestamp > bag_buffer_time_sec_)) {
                    break; 
                }
                auto serialized_msg = reader.read_next();
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);

                if (serialized_msg->topic_name == imu_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
                    imu_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                    message_buffer.push_back({get_time_sec(msg->header.stamp), msg, nullptr, nullptr, imu_topic});
                } else if (serialized_msg->topic_name == lid_topic) {
                    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
                    double header_stamp = 0.0;
                    if (config.lidar_type == AVIA) {
                        auto livox_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
                        livox_serialization.deserialize_message(&extracted_serialized_msg, livox_msg.get());
                        header_stamp = get_time_sec(livox_msg->header.stamp);
                        fast_lio_core_->p_pre_->process(std::move(livox_msg), cloud);
                    } else {
                        auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
                        pc2_serialization.deserialize_message(&extracted_serialized_msg, pc2_msg.get());
                        header_stamp = get_time_sec(pc2_msg->header.stamp);
                        fast_lio_core_->p_pre_->process(std::move(pc2_msg), cloud);
                    }
                    message_buffer.push_back({header_stamp, nullptr, cloud, nullptr, lid_topic});
                } else if (serialized_msg->topic_name == tf_topic || serialized_msg->topic_name == tf_static_topic) {
                    auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
                    tf_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                    if (!msg->transforms.empty()) {
                        message_buffer.push_back({get_time_sec(msg->transforms[0].header.stamp), nullptr, nullptr, msg, serialized_msg->topic_name});
                    }
                }
            }

            std::stable_sort(message_buffer.begin(), message_buffer.end());

            double process_until_time = message_buffer.back().timestamp - (bag_buffer_time_sec_ / 2.0);
            if (!reader.has_next()) {
                process_until_time = std::numeric_limits<double>::max();
            }

            while (!message_buffer.empty() && message_buffer.front().timestamp < process_until_time) {
                StampedMessage stamped_msg = message_buffer.front();
                message_buffer.pop_front();

                rosgraph_msgs::msg::Clock clock_msg;
                clock_msg.clock = get_ros_time(stamped_msg.timestamp);
                clock_pub_->publish(clock_msg);

                { // Push to buffers
                    std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);
                    if (stamped_msg.topic_name == imu_topic) {
                        fast_lio_core_->imu_buffer_.push_back(stamped_msg.imu_msg);
                    } else if (stamped_msg.topic_name == lid_topic) {
                        fast_lio_core_->lidar_buffer_.push_back(stamped_msg.lidar_msg);
                        fast_lio_core_->time_buffer_.push_back(stamped_msg.timestamp);
                    } else if (stamped_msg.topic_name == tf_topic) {
                        tf_broadcaster_->sendTransform(stamped_msg.tf_msg->transforms);
                    } else if (stamped_msg.topic_name == tf_static_topic) {
                        static_tf_broadcaster_->sendTransform(stamped_msg.tf_msg->transforms);
                    }
                }

                MeasureGroup meas;
                if (fast_lio_core_->sync_packages(meas)) {
                    FrameResult result = fast_lio_core_->process_frame(meas);
                    if (result.cloud.empty()) continue;

                    nav_msgs::msg::Odometry odom_msg;
                    geometry_msgs::msg::Quaternion quat;
                    fast_lio_core_->get_publish_odometry(odom_msg, quat);
                    odom_msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                    odom_msg.header.frame_id = map_frame_;

                    Eigen::Matrix4d pose_world = Eigen::Matrix4d::Identity();
                    pose_world.block<3, 3>(0, 0) = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z).toRotationMatrix();
                    pose_world(0, 3) = odom_msg.pose.pose.position.x;
                    pose_world(1, 3) = odom_msg.pose.pose.position.y;
                    pose_world(2, 3) = odom_msg.pose.pose.position.z;

                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(result.cloud, *cloud_world, pose_world);

                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_publish(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::copyPointCloud(*cloud_world, *cloud_to_publish);

                    sensor_msgs::msg::PointCloud2 pcd_msg;
                    pcl::toROSMsg(*cloud_to_publish, pcd_msg);
                    pcd_msg.header.stamp = odom_msg.header.stamp;
                    pcd_msg.header.frame_id = "body";

                    odomPcdCallback(std::make_shared<nav_msgs::msg::Odometry>(odom_msg),
                                    std::make_shared<sensor_msgs::msg::PointCloud2>(pcd_msg));

                    nav_msgs::msg::Path live_corrected_path;
                    live_corrected_path.header.frame_id = map_frame_;
                    live_corrected_path.header.stamp = odom_msg.header.stamp;
                    std::lock_guard<std::mutex> lock(keyframes_mutex_);
                    for(const auto& kf : keyframes_) {
                        live_corrected_path.poses.push_back(
                                poseEigToPoseStamped(kf.pose_corrected_eig_, map_frame_)
                                );
                    }
                    corrected_path_pub_->publish(live_corrected_path);


                    if (corrected_pcd_map_pub_->get_subscription_count() > 0 && !keyframes_.empty())
                    {
                        vis_count_++;
                        if (vis_count_ % map_publish_freq_ == 0)
                        {
                            pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
                            corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());
                            {
                                for (size_t i = 0; i < keyframes_.size(); ++i)
                                {
                                    *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
                                }
                            }
                            const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
                            corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
                        }

                    }
                    rate.sleep();
                }
            }
        }
    }
    else // Full bag read mode
    {
        RCLCPP_INFO(this->get_logger(), "Offline mode with FULL BAG reading.");
        std::vector<StampedMessage> all_messages;
        RCLCPP_INFO(this->get_logger(), "Reading all messages from bag...");
        while(reader.has_next())
        {
            auto serialized_msg = reader.read_next();
            rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);

            if (serialized_msg->topic_name == imu_topic)
            {
                auto msg = std::make_shared<sensor_msgs::msg::Imu>();
                imu_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                all_messages.push_back({get_time_sec(msg->header.stamp), msg, nullptr, nullptr, imu_topic});
            }
            else if (serialized_msg->topic_name == lid_topic)
            {
                PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
                double header_stamp = 0.0;
                if (config.lidar_type == AVIA) {
                    auto livox_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
                    livox_serialization.deserialize_message(&extracted_serialized_msg, livox_msg.get());
                    header_stamp = get_time_sec(livox_msg->header.stamp);
                    fast_lio_core_->p_pre_->process(std::move(livox_msg), cloud);
                } else {
                    auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
                    pc2_serialization.deserialize_message(&extracted_serialized_msg, pc2_msg.get());
                    header_stamp = get_time_sec(pc2_msg->header.stamp);
                    fast_lio_core_->p_pre_->process(std::move(pc2_msg), cloud);
                }
                all_messages.push_back({header_stamp, nullptr, cloud, nullptr, lid_topic});
            } else if (serialized_msg->topic_name == tf_topic || serialized_msg->topic_name == tf_static_topic) {
                auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
                tf_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                if (!msg->transforms.empty()) {
                    all_messages.push_back({get_time_sec(msg->transforms[0].header.stamp), nullptr, nullptr, msg, serialized_msg->topic_name});
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Read %zu total messages. Sorting...", all_messages.size());
        std::sort(all_messages.begin(), all_messages.end());
        RCLCPP_INFO(this->get_logger(), "Finished sorting messages. Starting processing loop.");

        // IMU Initialization
        deque<sensor_msgs::msg::Imu::ConstSharedPtr> init_imu_data;
        for (const auto& msg : all_messages) {
            if (msg.topic_name == imu_topic) {
                init_imu_data.push_back(msg.imu_msg);
                if (init_imu_data.size() >= INIT_IMU_COUNT) break;
            }
        }

        if (init_imu_data.size() < INIT_IMU_COUNT) {
            RCLCPP_ERROR(this->get_logger(), "Not enough IMU messages for initialization!");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);
            fast_lio_core_->imu_buffer_ = init_imu_data;
        }
        fast_lio_core_->initial_setup();
        RCLCPP_INFO(this->get_logger(), "IMU Initialized.");

        // Main Processing Loop
        for (const auto& msg : all_messages) {
            if (!rclcpp::ok()) break;
            {
                std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);

                rosgraph_msgs::msg::Clock clock_msg;
                clock_msg.clock = get_ros_time(msg.timestamp);
                clock_pub_->publish(clock_msg);

                if (msg.topic_name == imu_topic) {
                    fast_lio_core_->imu_buffer_.push_back(msg.imu_msg);
                } else if (msg.topic_name == lid_topic) {
                    fast_lio_core_->lidar_buffer_.push_back(msg.lidar_msg);
                    fast_lio_core_->time_buffer_.push_back(msg.timestamp);
                } else if (msg.topic_name == tf_topic) {
                    tf_broadcaster_->sendTransform(msg.tf_msg->transforms);
                } else if (msg.topic_name == tf_static_topic) {
                    static_tf_broadcaster_->sendTransform(msg.tf_msg->transforms);
                }
            }

            MeasureGroup meas;
            if (fast_lio_core_->sync_packages(meas)) {
                FrameResult result = fast_lio_core_->process_frame(meas);
                if (result.cloud.empty()) continue;

                nav_msgs::msg::Odometry odom_msg;
                geometry_msgs::msg::Quaternion quat;
                fast_lio_core_->get_publish_odometry(odom_msg, quat);
                odom_msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                odom_msg.header.frame_id = map_frame_;

                Eigen::Matrix4d pose_world = Eigen::Matrix4d::Identity();
                pose_world.block<3, 3>(0, 0) = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z).toRotationMatrix();
                pose_world(0, 3) = odom_msg.pose.pose.position.x;
                pose_world(1, 3) = odom_msg.pose.pose.position.y;
                pose_world(2, 3) = odom_msg.pose.pose.position.z;

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::transformPointCloud(result.cloud, *cloud_world, pose_world);

                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_publish(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::copyPointCloud(*cloud_world, *cloud_to_publish);

                sensor_msgs::msg::PointCloud2 pcd_msg;
                pcl::toROSMsg(*cloud_to_publish, pcd_msg);
                pcd_msg.header.stamp = odom_msg.header.stamp;
                pcd_msg.header.frame_id = "body";

                odomPcdCallback(std::make_shared<nav_msgs::msg::Odometry>(odom_msg),
                        std::make_shared<sensor_msgs::msg::PointCloud2>(pcd_msg));

                nav_msgs::msg::Path live_corrected_path;
                live_corrected_path.header.frame_id = map_frame_;
                live_corrected_path.header.stamp = odom_msg.header.stamp;
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for(const auto& kf : keyframes_) {
                    live_corrected_path.poses.push_back(
                            poseEigToPoseStamped(kf.pose_corrected_eig_, map_frame_)
                            );
                }
                corrected_path_pub_->publish(live_corrected_path);

                if (corrected_pcd_map_pub_->get_subscription_count() > 0 && !keyframes_.empty())
                {
                    vis_count_++;
                    if (vis_count_ % map_publish_freq_ == 0)
                    {
                        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
                        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());
                        {
                            for (size_t i = 0; i < keyframes_.size(); ++i)
                            {
                                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
                            }
                        }
                        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
                        corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
                    }
                }

                rate.sleep();
            }
        }
    }


    // Final post loop optimization
    if (offline_post_loop_optimization_)
    {
        RCLCPP_INFO(this->get_logger(), "Finished processing all messages. Starting offline BATCH loop closure and optimization...");
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            performLoopClosureForKf(i);
        }

        RCLCPP_INFO(this->get_logger(), "Performing final batch graph optimization...");
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            isam_handler_->update(gtsam_graph_, init_esti_);
            isam_handler_->update();
            gtsam_graph_.resize(0);
            init_esti_.clear();
        }

        for (int i = 0; i < 10; ++i) {
            isam_handler_->update();
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Finished processing all messages with INCREMENTAL loop closure.");
    }

    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        std::lock_guard<std::mutex> lock2(realtime_pose_mutex_);
        corrected_esti_ = isam_handler_->calculateEstimate();
        for (size_t i = 0; i < corrected_esti_.size(); ++i)
        {
            if (i < keyframes_.size()) {
                keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
            }
        }
    }

    if (offline_post_loop_optimization_) {
        RCLCPP_INFO(this->get_logger(), "Publishing final optimized path...");
        nav_msgs::msg::Path final_path;
        final_path.header.frame_id = map_frame_;
        final_path.header.stamp = this->get_clock()->now();
        if (!keyframes_.empty()) {
            // Use the last keyframe's stamp for update
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            final_path.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframes_.back().timestamp_ * 1e9));
        }

        for (size_t i = 0; i < corrected_esti_.size(); ++i) {
            final_path.poses.push_back(
                gtsamPoseToPoseStamped(corrected_esti_.at<gtsam::Pose3>(i), map_frame_)
            );
        }
        corrected_path_pub_->publish(final_path);
    }

    RCLCPP_INFO(this->get_logger(), "Offline processing finished. Final results are now available.");
    rclcpp::shutdown();
}

void FastLioSam::saveMapToBag(const std::string& path,
        const std::vector<PosePcd>& keyframes,
        std::mutex& keyframes_mutex)
{
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = path + "map_bag";
    storage_options.storage_id = "sqlite3";

    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    try {
        writer->open(storage_options);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open bag file for writing: %s", e.what());
        return;
    }

    const std::string pose_topic_name = "/keyframe_pose";
    rosbag2_storage::TopicMetadata pose_topic_metadata;
    pose_topic_metadata.name = pose_topic_name;
    pose_topic_metadata.type = "geometry_msgs/msg/PoseStamped";
    pose_topic_metadata.serialization_format = rmw_get_serialization_format();
    writer->create_topic(pose_topic_metadata);


    const std::string pcd_topic_name = "/keyframe_pcd";
    rosbag2_storage::TopicMetadata pcd_topic_metadata;
    pcd_topic_metadata.name = pcd_topic_name;
    pcd_topic_metadata.type = "sensor_msgs/msg/PointCloud2";
    pcd_topic_metadata.serialization_format = rmw_get_serialization_format();
    writer->create_topic(pcd_topic_metadata);


    {
        std::lock_guard<std::mutex> lock(keyframes_mutex);

        for (const auto& keyframe : keyframes)
        {
            // Convert timestamp
            rclcpp::Time time(static_cast<int64_t>(keyframe.timestamp_ * 1e9));

            // Write pose message
            auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>(
                poseEigToPoseStamped(keyframe.pose_corrected_eig_, map_frame_)
            );
            pose_msg->header.stamp = time;
            try {
                writer->write(*pose_msg, pose_topic_name, time);
            } catch (const std::exception& e) {
                 RCLCPP_ERROR(this->get_logger(), "Failed to write pose message to bag: %s", e.what());
            }

            // Write PCD message
            auto pcd_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(
                pclToPclRos(keyframe.pcd_, map_frame_)
            );
            pcd_msg->header.stamp = time;
             try {
                writer->write(*pcd_msg, pcd_topic_name, time);
            } catch (const std::exception& e) {
                 RCLCPP_ERROR(this->get_logger(), "Failed to write pcd message to bag: %s", e.what());
            }
        }
    }
}

FastLioSam::~FastLioSam()
{
    // save map
    if (save_map_bag_)
    {
        RCLCPP_INFO(this->get_logger(), "Saving result to bag file...");
        saveMapToBag(save_map_path_, keyframes_, keyframes_mutex_);
        RCLCPP_INFO(this->get_logger(), "\033[36;1mResult saved in .bag format!!!\033[0m");
    }

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
        pcl::io::savePCDFileASCII<PointType>(save_map_path_ + "map.pcd", *voxelized_map);
        RCLCPP_INFO(this->get_logger(), "\033[32;1mResult saved in .pcd format at %s !!!\033[0m", save_map_path_.c_str());
    }
}


void FastLioSam::loadParams()
{
    this->declare_parameter("basic.map_frame", "map");
    this->declare_parameter("basic.robot_frame", "robot");
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
    this->declare_parameter("result.save_map_path", ROOT_DIR);
    this->declare_parameter("result.save_map_bag", false);
    this->declare_parameter("result.save_in_kitti_format", false);
    this->declare_parameter("result.seq_name", "");
    this->declare_parameter("result.save_pose_yml", false);
    this->declare_parameter("result.yaml_file_name", "");
    this->declare_parameter("result.yaml_file_name_bkp", "");
    this->declare_parameter("result.bkp_dt", 1);
    this->declare_parameter("result.map_publish_freq", 10);
    this->declare_parameter("offline.bag_file", "");
    this->declare_parameter("offline.fast_lio_config", "mid360.yaml");
    this->declare_parameter("offline.post_loop_optimization", false);
    this->declare_parameter("offline.buffered_read", true);
    this->declare_parameter("offline.buffer_time_sec", 2.0);


    this->get_parameter("basic.map_frame", map_frame_);
    this->get_parameter("basic.robot_frame", robot_frame_);
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
    this->get_parameter("result.save_map_path", save_map_path_);
    this->get_parameter("result.save_map_bag", save_map_bag_);
    this->get_parameter("result.save_in_kitti_format", save_in_kitti_format_);
    this->get_parameter("result.seq_name", seq_name_);
    this->get_parameter("result.save_pose_yml", save_pose_yml_);
    this->get_parameter("result.yaml_file_name", yaml_file_name_);
    this->get_parameter("result.yaml_file_name_bkp", yaml_file_name_bkp_);
    this->get_parameter("result.bkp_dt", bkp_dt_);
    this->get_parameter("result.map_publish_freq", map_publish_freq_);
    this->get_parameter("offline.bag_file", bag_file_);
    this->get_parameter("offline.fast_lio_config", fast_lio_config_);
    this->get_parameter("offline.post_loop_optimization", offline_post_loop_optimization_);
    this->get_parameter("offline.buffered_read", offline_buffered_read_);
    this->get_parameter("offline.buffer_time_sec", bag_buffer_time_sec_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
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

    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

}

void FastLioSam::initSubscribers()
{
    odom_sub_ = std::make_unique<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "Odometry");
    pcd_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cloud_registered");

    sub_odom_pcd_sync_ = std::make_unique<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10),*odom_sub_, *pcd_sub_);

    sub_odom_pcd_sync_->registerCallback(std::bind(&FastLioSam::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));

    sub_save_flag_ = this->create_subscription<std_msgs::msg::String>("save_dir", 1, std::bind(&FastLioSam::saveFlagCallback, this, std::placeholders::_1));
}

void FastLioSam::initTimers()
{
    loop_timer_ = this->create_wall_timer(500ms, std::bind(&FastLioSam::loopTimerCallback, this));
    vis_timer_ = this->create_wall_timer(500ms, std::bind(&FastLioSam::visTimerCallback, this));
}

geometry_msgs::msg::TransformStamped FastLioSam::getTransformStamped(const tf2::Transform &transform, const rclcpp::Time &stamp, const std::string &frame_id, const std::string &child_frame_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Populate the TransformStamped message
    transform_stamped.header.stamp = stamp;
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
        transform_stamped = getTransformStamped(transform, odom_msg->header.stamp, map_frame_, robot_frame_);
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

            // If in offline mode and incremental optimization is selected, perform loop check now
            if (!bag_file_.empty() && !offline_post_loop_optimization_)
            {
                performLoopClosureForKf(keyframes_.size() - 1);
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
                    if (i < keyframes_.size()){
                        keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                    }
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
    if (keyframes_.empty() || !is_initialized_) { return; }

    // In online mode, we only check the latest keyframe
    performLoopClosureForKf(keyframes_.size() - 1);

    // The visualization part from the original function can be kept separate
    // as it is tied to the timer frequency, not the loop closure logic itself.
    if (loop_closure_->getClosestKeyframeidx() >= 0) {
        debug_src_pub_->publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
        debug_dst_pub_->publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
        debug_fine_aligned_pub_->publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
    }
    loop_added_flag_vis_ = loop_added_flag_; // Signal vis timer to update
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
        corrected_path.header.frame_id = map_frame_;
        corrected_path.header.stamp = this->get_clock()->now();
        if (!keyframes_.empty()) {
            // Use the last keyframe's stamp for update
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            corrected_path.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframes_.back().timestamp_ * 1e9));
        }

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
        pcl::io::savePCDFileASCII<PointType>(save_map_path_ + seq_name_ + "_map.pcd", *voxelized_map);
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
