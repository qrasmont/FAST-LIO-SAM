#pragma once
///// Common Headers
#include <string>
///// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h> // CreateQuaternionFromRPY
#include <tf2_eigen/tf2_eigen.hpp>     // tf2 <-> Eigen
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
///// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
///// Eigen
#include <Eigen/Eigen>
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

typedef pcl::PointXYZI PointType;

//////////////////////////////////////////////////////////////////////
///// Conversions
inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in)
{
    double r, p, y;
    tf2::Matrix3x3 mat;
    mat.setValue(pose_eig_in(0, 0), pose_eig_in(0, 1), pose_eig_in(0, 2),
                 pose_eig_in(1, 0), pose_eig_in(1, 1), pose_eig_in(1, 2),
                 pose_eig_in(2, 0), pose_eig_in(2, 1), pose_eig_in(2, 2));
    mat.getRPY(r, p, y);
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(r, p, y),
                        gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}

inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in)
{
    Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
    tf2::Quaternion quat;
    quat.setRPY(gtsam_pose_in.rotation().roll(),
                gtsam_pose_in.rotation().pitch(),
                gtsam_pose_in.rotation().yaw());
    Eigen::Matrix3d tmp_rot_mat = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z()).toRotationMatrix();
    pose_eig_out.block<3, 3>(0, 0) = tmp_rot_mat;
    pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
    pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
    pose_eig_out(2, 3) = gtsam_pose_in.translation().z();
    return pose_eig_out;
}

inline geometry_msgs::msg::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                                                            const std::string &frame_id = "map")
{
    double r, p, y;
    tf2::Matrix3x3 mat;
    mat.setValue(pose_eig_in(0, 0), pose_eig_in(0, 1), pose_eig_in(0, 2),
                 pose_eig_in(1, 0), pose_eig_in(1, 1), pose_eig_in(1, 2),
                 pose_eig_in(2, 0), pose_eig_in(2, 1), pose_eig_in(2, 2));
    mat.getRPY(r, p, y);
    tf2::Quaternion quat;
    quat.setRPY(r, p, y);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    return pose;
}

inline geometry_msgs::msg::PoseStamped gtsamPoseToPoseStamped(const gtsam::Pose3 &gtsam_pose_in,
                                                              const std::string &frame_id = "map")
{
    tf2::Quaternion quat;
    quat.setRPY(gtsam_pose_in.rotation().roll(),
                gtsam_pose_in.rotation().pitch(),
                gtsam_pose_in.rotation().yaw());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = gtsam_pose_in.translation().x();
    pose.pose.position.y = gtsam_pose_in.translation().y();
    pose.pose.position.z = gtsam_pose_in.translation().z();
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    return pose;
}

inline tf2::Transform poseEigToROSTf2(const Eigen::Matrix4d &pose)
{
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    transform.setRotation(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    return transform;
}

inline tf2::Transform poseStampedToROSTf2(const geometry_msgs::msg::PoseStamped &pose)
{
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(pose.pose.position.x,
                                     pose.pose.position.y,
                                     pose.pose.position.z));
    transform.setRotation(tf2::Quaternion(pose.pose.orientation.x,
                                          pose.pose.orientation.y,
                                          pose.pose.orientation.z,
                                          pose.pose.orientation.w));
    return transform;
}


template <typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud,
                                                 const std::string &frame_id = "map")
{
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.frame_id = frame_id;
    return cloud_ros;
}

///// Transformation
template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf)
{
    if (cloud_in.empty())
    {
        return cloud_in;
    }
    pcl::PointCloud<T> pcl_out = cloud_in;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
    return pcl_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in,
                                       const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    auto pcd_out = std::make_shared<pcl::PointCloud<PointType>>();
    voxelgrid.setInputCloud(pcd_in.makeShared());
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
                                       const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    auto pcd_out = std::make_shared<pcl::PointCloud<PointType>>();
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

