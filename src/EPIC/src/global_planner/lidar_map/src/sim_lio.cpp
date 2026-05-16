/*
If you need to replace with other point cloud data structures, please
re-implement the following interfaces:

1. getDisToOcc: Returns the distance from the specified point to the nearest
obstacle in the map.
2. KNN: Nearest neighbor search. boxSearch: Region search.
3. updateCloudMapOdometry: Point cloud map update. 
4. LIOInterfaceData::Ptr ld: Current frame world coordinate point clouds and
lidar-odometry.

If you need to integrate EPIC with Lidar SLAM algorithm and shares
memory, thread mutual exclusion should be noted.
*/
#include "visualization_msgs/Marker.h"
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
namespace fast_planner {

bool LIOInterface::isStaticTransform(const std::string& target_frame,
                                      const std::string& source_frame) {
  try {
    auto t1 = tf_buffer_->lookupTransform(target_frame, source_frame,
                                           ros::Time(0), ros::Duration(5.0));
    ros::Duration(1.0).sleep();
    auto t2 = tf_buffer_->lookupTransform(target_frame, source_frame,
                                           ros::Time(0), ros::Duration(1.0));

    // Compare using Eigen
    Eigen::Vector3d p1(t1.transform.translation.x,
                       t1.transform.translation.y,
                       t1.transform.translation.z);
    Eigen::Vector3d p2(t2.transform.translation.x,
                       t2.transform.translation.y,
                       t2.transform.translation.z);

    Eigen::Quaterniond q1(t1.transform.rotation.w, t1.transform.rotation.x,
                          t1.transform.rotation.y, t1.transform.rotation.z);
    Eigen::Quaterniond q2(t2.transform.rotation.w, t2.transform.rotation.x,
                          t2.transform.rotation.y, t2.transform.rotation.z);

    double pos_diff = (p1 - p2).norm();
    double rot_diff = q1.angularDistance(q2);

    return (pos_diff < 1e-6 && rot_diff < 1e-6);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("[LIOInterface] Cannot check if %s->%s is static: %s",
             source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

void LIOInterface::initializeTransform(const std::string& map_frame,
                                       const std::string& body_frame,
                                       const std::string& cloud_frame) {
  map_frame_ = map_frame;
  body_frame_ = body_frame;
  cloud_frame_ = cloud_frame;

  // Case 0: Same frame - no transform needed
  if (cloud_frame_ == map_frame_) {
    transform_mode_ = TransformMode::NONE;
    ROS_INFO("[LIOInterface] Cloud frame '%s' matches map frame '%s', no transform needed",
             cloud_frame_.c_str(), map_frame_.c_str());
    return;
  }

  // Case 1: Check if cloud -> map is static
  ROS_INFO("[LIOInterface] Checking if %s -> %s is static (waiting 1 sec)...",
           cloud_frame_.c_str(), map_frame_.c_str());
  if (isStaticTransform(map_frame_, cloud_frame_)) {
    try {
      geometry_msgs::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(map_frame_, cloud_frame_, ros::Time(0), ros::Duration(1.0));
      Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
      T_map_cloud_static_ = transform_eigen.matrix().cast<float>();

      transform_mode_ = TransformMode::CLOUD_IN_WORLD;
      ROS_INFO("[LIOInterface] Using CLOUD_IN_WORLD mode: %s -> %s",
               cloud_frame_.c_str(), map_frame_.c_str());
      return;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[LIOInterface] Failed to lookup static transform %s -> %s: %s",
               cloud_frame_.c_str(), map_frame_.c_str(), ex.what());
    }
  }

  // Case 2: Check if cloud -> body is static
  ROS_INFO("[LIOInterface] Checking if %s -> %s is static (waiting 1 sec)...",
           cloud_frame_.c_str(), body_frame_.c_str());
  if (isStaticTransform(body_frame_, cloud_frame_)) {
    try {
      geometry_msgs::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(body_frame_, cloud_frame_, ros::Time(0), ros::Duration(1.0));
      Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
      T_body_cloud_ = transform_eigen.matrix().cast<float>();

      transform_mode_ = TransformMode::CLOUD_IN_LOCAL;
      ROS_INFO("[LIOInterface] Using CLOUD_IN_LOCAL mode: %s -> %s -> %s",
               cloud_frame_.c_str(), body_frame_.c_str(), map_frame_.c_str());
      return;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[LIOInterface] Failed to lookup static transform %s -> %s: %s",
               cloud_frame_.c_str(), body_frame_.c_str(), ex.what());
    }
  }

  // Case 3: Neither is static - error
  ROS_FATAL("[LIOInterface] Neither %s->%s nor %s->%s is static. Cannot proceed.",
            cloud_frame_.c_str(), map_frame_.c_str(),
            cloud_frame_.c_str(), body_frame_.c_str());
  ros::shutdown();
}

double LIOInterface::getDisToOcc(const PointType &pt) {
  PointVector nes_pts;
  vector<float> diss;
  KNN(pt, 1, nes_pts, diss);
  if (nes_pts.size() == 0)
    return 10.0;
  else
    return sqrt(diss[0]);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3d &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3f &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
void LIOInterface::KNN(const PointType &pt, int k, PointVector &pts,
                       vector<float> &dis) {
  ikd_Tree_map.Nearest_Search(pt, k, pts, dis, 10.0);
}
void LIOInterface::boxSearch(const Eigen::Vector3f &min_bd,
                             const Eigen::Vector3f &max_bd, PointVector &pts) {
  BoxPointType boxpoint;
  for (int i = 0; i < 3; i++) {
    boxpoint.vertex_min[i] = min_bd(i);
    boxpoint.vertex_max[i] = max_bd(i);
  }
  ikd_Tree_map.Box_Search(boxpoint, pts);
}
void LIOInterface::updateCloudMapOdometry(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom_) {
  ld_->map_update = true;
  static Eigen::Vector3f last_lidar_pose(0, 0, 0);
  Eigen::Vector3f lidar_pos_(odom_->pose.pose.position.x,
                             odom_->pose.pose.position.y,
                             odom_->pose.pose.position.z);
  Eigen::Vector3f lidar_vel_(odom_->twist.twist.linear.x,
                             odom_->twist.twist.linear.y,
                             odom_->twist.twist.linear.z);
  last_lidar_pose = lidar_pos_;
  ld_->lidar_pose_ = lidar_pos_;
  ld_->lidar_vel_ = lidar_vel_;
  Eigen::AngleAxisf y_axis_angle(M_PI / 180.0 * lp_->lidar_pitch_,
                                 Eigen::Vector3f::UnitY());
  Eigen::Quaternionf q_y(y_axis_angle);
  ld_->lidar_q_ = Eigen::Quaternionf(odom_->pose.pose.orientation.w,
                                     odom_->pose.pose.orientation.x,
                                     odom_->pose.pose.orientation.y,
                                     odom_->pose.pose.orientation.z) *
                  q_y;

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(*msg, cloud_input);

  // ROS_INFO_THROTTLE(1.0, "[LIOInterface] Input cloud size: %lu points", cloud_input.points.size());

  // OPTIMIZATION: Voxel filter BEFORE transformation (process fewer points)
  ros::Time start = ros::Time::now();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.4, 0.4, 0.4); // 0.1, 0.1, 0.1
  vg.setInputCloud(cloud_input.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*filtered_points);

  // ROS_INFO_THROTTLE(1.0, "[LIOInterface] After voxel filter: %lu points (took %.2fms)",
  //                   filtered_points->points.size(),
  //                   (ros::Time::now() - start).toSec() * 1000.0);

  if (filtered_points->points.empty()) {
    return;
  }

  // Compute T_map_body from odometry (used by all modes)
  Eigen::Matrix4f T_map_body = Eigen::Matrix4f::Identity();
  T_map_body.block<3, 1>(0, 3) = lidar_pos_;
  Eigen::Quaternionf q_map_body(odom_->pose.pose.orientation.w,
                                 odom_->pose.pose.orientation.x,
                                 odom_->pose.pose.orientation.y,
                                 odom_->pose.pose.orientation.z);
  T_map_body.block<3, 3>(0, 0) = q_map_body.toRotationMatrix();

  // Compute T_body_cloud based on transform mode (for CropBox)
  Eigen::Matrix4f T_body_cloud;
  if (transform_mode_ == TransformMode::CLOUD_IN_LOCAL) {
    // cloud -> body is static (cached)
    T_body_cloud = T_body_cloud_;
  } else if (transform_mode_ == TransformMode::CLOUD_IN_WORLD) {
    // T_body_cloud = T_body_map * T_map_cloud = T_map_body.inverse() * T_map_cloud_static_
    T_body_cloud = T_map_body.inverse() * T_map_cloud_static_;
  } else {
    // NONE: cloud == map, use T_body_map
    T_body_cloud = T_map_body.inverse();
  }

  // CropBox filtering in body frame
  if (lp_->use_cropbox_) {
    // Transform points from cloud frame to body frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_body(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*filtered_points, *cloud_in_body, T_body_cloud);

    // Apply CropBox filter in body frame
    pcl::CropBox<pcl::PointXYZ> cropbox;
    cropbox.setInputCloud(cloud_in_body);
    cropbox.setMin(Eigen::Vector4f(lp_->cropbox_min_.x(), lp_->cropbox_min_.y(),
                                    lp_->cropbox_min_.z(), 1.0));
    cropbox.setMax(Eigen::Vector4f(lp_->cropbox_max_.x(), lp_->cropbox_max_.y(),
                                    lp_->cropbox_max_.z(), 1.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_body(new pcl::PointCloud<pcl::PointXYZ>);
    cropbox.filter(*cropped_body);

    // Transform back to cloud frame
    Eigen::Matrix4f T_cloud_body = T_body_cloud.inverse();
    pcl::transformPointCloud(*cropped_body, *filtered_points, T_cloud_body);

    if (filtered_points->points.empty()) {
      return;
    }
  }

  // Apply frame transformation to map frame based on transform mode
  if (transform_mode_ == TransformMode::CLOUD_IN_LOCAL) {
    // T_map_cloud = T_map_body * T_body_cloud_
    Eigen::Matrix4f T_map_cloud = T_map_body * T_body_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*filtered_points, *transformed_cloud, T_map_cloud);
    filtered_points = transformed_cloud;
  } else if (transform_mode_ == TransformMode::CLOUD_IN_WORLD) {
    // T_map_cloud is static (cached)
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*filtered_points, *transformed_cloud, T_map_cloud_static_);
    filtered_points = transformed_cloud;
  }
  // TransformMode::NONE - no transformation needed

  // BUGFIX: Store the processed cloud in ld_->lidar_cloud_ (was missing in frame transform version)
  ld_->lidar_cloud_ = *filtered_points;
  PointVector pcl_map = filtered_points->points;

  if (ld_->first_map_flag_) {
    // this->ikd_Tree_map(0.3,0.6,0.2);
    this->ikd_Tree_map.set_downsample_param(0.1);
    this->ikd_Tree_map.Build(pcl_map);
    ld_->first_map_flag_ = false;
  } else {
    this->ikd_Tree_map.Add_Points(pcl_map, true);
  }
  ros::Time ikd_update_end_stamp = ros::Time::now();
}

} // namespace fast_planner