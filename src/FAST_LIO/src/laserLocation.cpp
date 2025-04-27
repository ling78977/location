// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "use-ikfom.hpp"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <Python.h>
#include <cmath>
#include <csignal>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ikd-Tree/ikd_Tree.h>
#include <iostream>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <math.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omp.h>
#include <ostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <so3_math.h>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0,
       kdtree_delete_time = 0.0;
// double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN],
//     s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN],
//     s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0,
    kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false,
     extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0,
       filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0,
       lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
    laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool is_first_lidar = true;

vector<vector<int>> pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
PointCloudXYZI::Ptr globalMap(new PointCloudXYZI());
// PointCloudXYZI::Ptr testLocation(new PointCloudXYZI());
// 初始化配准定位需要的一帧点云
PointCloudXYZI::Ptr initLocationCloud(new PointCloudXYZI());
// PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
// PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
// pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
state_ikfom last_key_state_point;
vect3 pos_lid;

nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig) {
  flg_exit = true;
  std::cout << "catch sig %d" << sig << std::endl;
  sig_buffer.notify_all();
  rclcpp::shutdown();
}

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void points_cache_collect() {
  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);
}

// BoxPointType LocalMap_Points;
// bool Localmap_Initialized = false;
// void lasermap_fov_segment() {
//   cub_needrm.clear();
//   kdtree_delete_counter = 0;
//   kdtree_delete_time = 0.0;
//   pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
//   V3D pos_LiD = pos_lid;
//   if (!Localmap_Initialized) {
//     for (int i = 0; i < 3; i++) {
//       LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
//       LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
//     }
//     Localmap_Initialized = true;
//     return;
//   }
//   float dist_to_map_edge[3][2];
//   bool need_move = false;
//   for (int i = 0; i < 3; i++) {
//     dist_to_map_edge[i][0] = fabs(pos_LiD(i) -
//     LocalMap_Points.vertex_min[i]); dist_to_map_edge[i][1] = fabs(pos_LiD(i)
//     - LocalMap_Points.vertex_max[i]); if (dist_to_map_edge[i][0] <=
//     MOV_THRESHOLD * DET_RANGE ||
//         dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
//       need_move = true;
//   }
//   if (!need_move)
//     return;
//   BoxPointType New_LocalMap_Points, tmp_boxpoints;
//   New_LocalMap_Points = LocalMap_Points;
//   float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 *
//   0.9,
//                        double(DET_RANGE * (MOV_THRESHOLD - 1)));
//   for (int i = 0; i < 3; i++) {
//     tmp_boxpoints = LocalMap_Points;
//     if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
//       New_LocalMap_Points.vertex_max[i] -= mov_dist;
//       New_LocalMap_Points.vertex_min[i] -= mov_dist;
//       tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
//       cub_needrm.push_back(tmp_boxpoints);
//     } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
//       New_LocalMap_Points.vertex_max[i] += mov_dist;
//       New_LocalMap_Points.vertex_min[i] += mov_dist;
//       tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
//       cub_needrm.push_back(tmp_boxpoints);
//     }
//   }
//   LocalMap_Points = New_LocalMap_Points;

//   points_cache_collect();
//   double delete_begin = omp_get_wtime();
//   if (cub_needrm.size() > 0)
//     kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
//   kdtree_delete_time = omp_get_wtime() - delete_begin;
// }
bool need_init_location = true;
double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
  if (initLocationCloud == nullptr || initLocationCloud->empty()) {
    p_pre->process(msg, initLocationCloud);
  }
  if (need_init_location) {
    // 正在初始化定位当中，直接跳过
    return;
  }
  mtx_buffer.lock();
  double cur_time = get_time_sec(msg->header.stamp);
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (!is_first_lidar && cur_time < last_timestamp_lidar) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    lidar_buffer.clear();
  }
  if (is_first_lidar) {
    is_first_lidar = false;
  }
  last_timestamp_lidar = cur_time;

  if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
      !imu_buffer.empty() && !lidar_buffer.empty()) {
    printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
           last_timestamp_imu, last_timestamp_lidar);
  }

  if (time_sync_en && !timediff_set_flg &&
      abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
      !imu_buffer.empty()) {
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    printf("Self sync IMU and LiDAR, time diff is %.10lf \n",
           timediff_lidar_wrt_imu);
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(last_timestamp_lidar);
  // static bool save=true;
  // if(save){
  //   pcl::PCDWriter pcd_writer;
  //   // cout << "current scan saved to /PCD/" << file_name << endl;
  //   pcd_writer.writeBinary(std::string(ROOT_DIR)+"/PCD/test_location.pcd",
  //   *ptr);
  // }

  // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  if (need_init_location) {
    // 正在初始化定位当中，直接跳过
    return;
  }
  publish_count++;
  // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  msg->header.stamp =
      get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
    msg->header.stamp = rclcpp::Time(timediff_lidar_wrt_imu +
                                     get_time_sec(msg_in->header.stamp));
  }

  double timestamp = get_time_sec(msg->header.stamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool sync_packages(MeasureGroup &meas) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 1) // time too little
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      std::cerr << "Too few input point cloud!\n";
    } else if (meas.lidar->points.back().curvature / double(1000) <
               0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time +
                       meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime +=
          (meas.lidar->points.back().curvature / double(1000) -
           lidar_mean_scantime) /
          scan_num;
    }

    meas.lidar_end_time = lidar_end_time;

    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    if (imu_time > lidar_end_time)
      break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }
  // std::cout<<"一个观测包里有多少个imu:"<<meas.imu.size()<<std::endl;
  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;

  return true;
}

// int process_increments = 0;
// void map_incremental() {
//   PointVector PointToAdd;
//   PointVector PointNoNeedDownsample;
//   PointToAdd.reserve(feats_down_size);
//   PointNoNeedDownsample.reserve(feats_down_size);
//   for (int i = 0; i < feats_down_size; i++) {
//     /* transform to world frame */
//     pointBodyToWorld(&(feats_down_body->points[i]),
//                      &(feats_down_world->points[i]));
//     /* decide if need add to map */
//     if (!Nearest_Points[i].empty() && flg_EKF_inited) {
//       const PointVector &points_near = Nearest_Points[i];
//       bool need_add = true;
//       BoxPointType Box_of_Point;
//       PointType downsample_result, mid_point;
//       mid_point.x = floor(feats_down_world->points[i].x /
//       filter_size_map_min) *
//                         filter_size_map_min +
//                     0.5 * filter_size_map_min;
//       mid_point.y = floor(feats_down_world->points[i].y /
//       filter_size_map_min) *
//                         filter_size_map_min +
//                     0.5 * filter_size_map_min;
//       mid_point.z = floor(feats_down_world->points[i].z /
//       filter_size_map_min) *
//                         filter_size_map_min +
//                     0.5 * filter_size_map_min;
//       float dist = calc_dist(feats_down_world->points[i], mid_point);
//       if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
//           fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
//           fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
//         PointNoNeedDownsample.push_back(feats_down_world->points[i]);
//         continue;
//       }
//       for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
//         if (points_near.size() < NUM_MATCH_POINTS)
//           break;
//         if (calc_dist(points_near[readd_i], mid_point) < dist) {
//           need_add = false;
//           break;
//         }
//       }
//       if (need_add)
//         PointToAdd.push_back(feats_down_world->points[i]);
//     } else {
//       PointToAdd.push_back(feats_down_world->points[i]);
//     }
//   }

//   double st_time = omp_get_wtime();
//   add_point_size = ikdtree.Add_Points(PointToAdd, true);
//   ikdtree.Add_Points(PointNoNeedDownsample, false);
//   add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
//   // kdtree_incremental_time = omp_get_wtime() - st_time;
// }

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudFull) {
  if (scan_pub_en) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                       : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                          &laserCloudWorld->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
  }
}

void publish_effect_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudEffect) {
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
  for (int i = 0; i < effct_feat_num; i++) {
    RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}
sensor_msgs::msg::PointCloud2 global_map_msg;
void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
                     pubLaserCloudMap) {
  if (global_map_msg.data.empty()) {
    return;
  }

  // sensor_msgs::msg::PointCloud2 laserCloudmsg;
  // pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
  // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  global_map_msg.header.stamp = get_ros_time(lidar_end_time);
  global_map_msg.header.frame_id = "camera_init";
  pubLaserCloudMap->publish(global_map_msg);
}

// void save_to_pcd() {
//   pcl::PCDWriter pcd_writer;
//   pcd_writer.writeBinary(std::string(ROOT_DIR) + "/PCD/scans.pcd",
//                          *pcl_wait_pub);

//   PointVector().swap(ikdtree.PCL_Storage);
//   ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
//   featsFromMap->clear();
//   featsFromMap->points = ikdtree.PCL_Storage;
//   pcd_writer.writeBinary(std::string(ROOT_DIR) + "/PCD/ikd.pcd",
//   *featsFromMap);
// }

template <typename T> void set_posestamp(T &out) {
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);
  out.pose.orientation.x = geoQuat.x;
  out.pose.orientation.y = geoQuat.y;
  out.pose.orientation.z = geoQuat.z;
  out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const rclcpp::Publisher<
                          nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                      std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  set_posestamp(odomAftMapped.pose);
  pubOdomAftMapped->publish(odomAftMapped);
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++) {
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "camera_init";
  trans.child_frame_id = "body";
  trans.header.stamp = get_ros_time(lidar_end_time);
  trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
  trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
  trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
  trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
  trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
  trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
  trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
  tf_br->sendTransform(trans);
}

void h_share_model(state_ikfom &s,
                   esekfom::dyn_share_datastruct<double> &ekfom_data) {
  double match_start = omp_get_wtime();
  laserCloudOri->clear();
  corr_normvect->clear();
  total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (int i = 0; i < feats_down_size; i++) {
    PointType &point_body = feats_down_body->points[i];
    PointType &point_world = feats_down_world->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = Nearest_Points[i];

    if (ekfom_data.converge) {
      /** Find the closest surfaces in the map **/
      ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                             pointSearchSqDis);
      point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false
                               : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5
                                   ? false
                                   : true;
    }

    if (!point_selected_surf[i])
      continue;

    VF(4) pabcd;
    point_selected_surf[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f)) {
      float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                  pabcd(2) * point_world.z + pabcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s > 0.9) {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i] = abs(pd2);
      }
    }
  }

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++) {
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    std::cerr << "No Effective Points!" << std::endl;
    // ROS_WARN("No Effective Points! \n");
    return;
  }

  res_mean_last = total_residual / effct_feat_num;
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
  ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
  ekfom_data.h.resize(effct_feat_num);

  for (int i = 0; i < effct_feat_num; i++) {
    const PointType &laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType &norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (extrinsic_est_en) {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() *
            C); // s.rot.conjugate()*norm_vec);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;
  }
  solve_time += omp_get_wtime() - solve_start_;
}
// 根据最新估计位姿  增量添加点云到map
bool init_ikdtree() {
  // 加载读取点云数据到cloud中
  string all_points_dir(string(string(ROOT_DIR) + "PCD/") + "ikd.pcd");
  // string all_points_dir2(string(string(ROOT_DIR) + "PCD/") +
  //                        "test_location.pcd");
  if (pcl::io::loadPCDFile<PointType>(all_points_dir, *globalMap) == -1) {
    PCL_ERROR("Read file fail!\n");
    return false;
  }
  // if (pcl::io::loadPCDFile<PointType>(all_points_dir, *globalMap) == -1 ||
  //     pcl::io::loadPCDFile<PointType>(all_points_dir2, *testLocation) == -1)
  //     {
  //   PCL_ERROR("Read file fail!\n");
  //   return false;
  // }

  ikdtree.set_downsample_param(0.1);
  ikdtree.Build(globalMap->points);
  std::cout << "---- ikdtree size: " << ikdtree.size() << std::endl;
  return true;
}

class LaserLocationNode : public rclcpp::Node {
public:
  LaserLocationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("laser_mapping", options) {
    this->declare_parameter<int>("max_iteration", 4);
    this->declare_parameter<string>("map_file_path", "");
    this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<bool>("common.time_sync_en", false);
    this->declare_parameter<bool>("common.debug_cloud_en", false);
    this->declare_parameter<bool>("publish.effect_map_en", false);
    this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    this->declare_parameter<double>("filter_size_corner", 0.5);
    this->declare_parameter<double>("filter_size_surf", 0.5);
    this->declare_parameter<bool>("publish.scan_publish_en", true);
    this->declare_parameter<double>("filter_size_map", 0.1);
    this->declare_parameter<double>("cube_side_length", 200.);
    this->declare_parameter<float>("mapping.det_range", 300.);
    this->declare_parameter<double>("mapping.fov_degree", 180.);
    this->declare_parameter<double>("mapping.gyr_cov", 0.1);
    this->declare_parameter<double>("mapping.acc_cov", 0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    this->declare_parameter<double>("preprocess.blind", 0.01);
    this->declare_parameter<int>("preprocess.lidar_type", AVIA);
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.timestamp_unit", US);
    this->declare_parameter<int>("preprocess.scan_rate", 10);
    this->declare_parameter<int>("point_filter_num", 2);
    this->declare_parameter<bool>("feature_extract_enable", false);
    this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    this->declare_parameter<vector<double>>("mapping.extrinsic_T",
                                            vector<double>());
    this->declare_parameter<vector<double>>("mapping.extrinsic_R",
                                            vector<double>());
    this->get_parameter_or<bool>("common.debug_cloud_en", debug_cloud_, false);
    this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
    this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
    this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    this->get_parameter_or<string>("map_file_path", map_file_path, "");
    this->get_parameter_or<string>("common.lid_topic", lid_topic,
                                   "/livox/lidar");
    this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
    this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
    this->get_parameter_or<double>("common.time_offset_lidar_to_imu",
                                   time_diff_lidar_to_imu, 0.0);
    this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min,
                                   0.5);
    this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min,
                                   0.5);
    this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.1);
    this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
    this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
    this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
    this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
    this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
    this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
    this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
    this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
    this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type,
                                AVIA);
    this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
    this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit,
                                US);
    this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
    this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
    this->get_parameter_or<bool>("feature_extract_enable",
                                 p_pre->feature_enabled, false);
    this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en,
                                 true);
    this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT,
                                           vector<double>());
    this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR,
                                           vector<double>());
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    // downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
    //                               filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    fill(epsi, epsi + 23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS,
                      epsi);

    /*** debug record ***/

    /*** ROS subscribe initialization ***/
    if (p_pre->lidar_type == AVIA) {
      sub_pcl_livox_ =
          this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
              lid_topic, 20, livox_pcl_cbk);
    } else {
      sub_pcl_livox_ =
          this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
              lid_topic, 20, livox_pcl_cbk);
    }

    pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 20);
    pubLaserCloudEffect_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected",
                                                              20);
    pubOdomAftMapped_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pubLaserCloudMap_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
    if (!init_ikdtree()) {
      return;
    }
    p_imu->is_location = true;
    pcl::toROSMsg(*globalMap, global_map_msg);

    RCLCPP_INFO(this->get_logger(),
                "Node init finished.Start init location...");
    // 等待第一帧雷达数据到达
    RCLCPP_INFO(this->get_logger(), "等待第一帧雷达数据到达...");
    while (initLocationCloud == nullptr || initLocationCloud->empty()) {
      rclcpp::spin_some(this->get_node_base_interface());
      this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.1));
    }
    RCLCPP_INFO(this->get_logger(), "第一帧雷达数据到达,开始配准...");
    if (!init_location()) {
      RCLCPP_ERROR(this->get_logger(), "初始化位置失败！");
      return;
    }
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                   filter_size_surf_min);
    RCLCPP_INFO(this->get_logger(), "配准成功...");
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10,
                                                                imu_cbk);

    auto map_period_ms =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    map_pub_timer_ = rclcpp::create_timer(
        this, this->get_clock(), map_period_ms,
        std::bind(&LaserLocationNode::map_publish_callback, this));

    //------------------------------------------------------------------------------------------------------

    auto period_ms =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    timer_ = rclcpp::create_timer(
        this, this->get_clock(), period_ms,
        std::bind(&LaserLocationNode::timer_callback, this));
  }

  ~LaserLocationNode() {}

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudEffect_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;

private:
  void timer_callback() {
    if (!sync_packages(Measures)) {
      return;
    }
    if (flg_first_scan) {
      first_lidar_time = Measures.lidar_beg_time;
      p_imu->first_lidar_time = first_lidar_time;
      flg_first_scan = false;
      return;
    }
    // double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

    // match_time = 0;
    // kdtree_search_time = 0.0;
    // solve_time = 0;
    // solve_const_H_time = 0;
    // svd_time = 0;
    // t0 = omp_get_wtime();

    p_imu->Process(Measures, kf, feats_undistort, pubOdomAftMapped_,
                   tf_broadcaster_);
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    if (feats_undistort->empty() || (feats_undistort == NULL)) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }

    flg_EKF_inited =
        (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
    /*** Segment the map in lidar FOV ***/
    // lasermap_fov_segment();

    /*** downsample the feature points in a scan ***/
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);
    // t1 = omp_get_wtime();
    feats_down_size = feats_down_body->points.size();

    int featsFromMapNum = ikdtree.validnum();
    kdtree_size_st = ikdtree.size();

    /*** ICP and iterated Kalman filter update ***/
    if (feats_down_size < 5) {
      RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
      return;
    }

    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);

    pointSearchInd_surf.resize(feats_down_size);
    Nearest_Points.resize(feats_down_size);
    int rematch_num = 0;
    bool nearest_search_en = true; //

    // t2 = omp_get_wtime();

    /*** iterated state estimation ***/
    double t_update_start = omp_get_wtime();
    double solve_H_time = 0;
    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];

    // double t_update_end = omp_get_wtime();

    // /******* Publish odometry *******/
    publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

    /******* Publish points *******/

    if (scan_pub_en)
      publish_frame_world(pubLaserCloudFull_);
    if (effect_pub_en)
      publish_effect_world(pubLaserCloudEffect_);
  }
  void map_publish_callback() {
    // if (map_pub_en)
    publish_map(pubLaserCloudMap_);
  }
  // clang-format off
  /*
  |----------------------------|
  |                            |
  |    o(4)    o(1)    o(6)    |
  |                            |
  |    o(7)    o(0)    o(3)    |
  |                            |
  |    o(2)    o(5)    o(8)    |
  |                            |  
  |----------------------------|
  */
  //中心点为地图原点，点与点之间上下左右默认间隔1m，在每个点默认以45°的角度分辨率依次旋转进行点云配准
  //直到匹配成功或者所有点都匹配失败退出，理论上在原点默认半径2m的圆周内都能够成功初始化位置
  // clang-format on
  bool init_location() {
    downSizeFilterSurf.setLeafSize(0.5, 0.5, 0.5);
    downSizeFilterSurf.setInputCloud(initLocationCloud);
    downSizeFilterSurf.filter(*initLocationCloud);
    // 储存转换到世界坐标系下的点
    PointCloudXYZI::Ptr initLocationCloudWorld(new PointCloudXYZI());
    initLocationCloudWorld->resize(initLocationCloud->size());
    // clang-format off
    std::vector<MTK::vect<3, double>> positions(
        {
         MTK::vect<3, double>({           0,              0, 0}),
         MTK::vect<3, double>({ spacing_dis_,             0, 0}),
         MTK::vect<3, double>({-spacing_dis_,  spacing_dis_, 0}),
         MTK::vect<3, double>({            0, -spacing_dis_, 0}),
         MTK::vect<3, double>({ spacing_dis_,  spacing_dis_, 0}),
         MTK::vect<3, double>({-spacing_dis_,             0, 0}),
         MTK::vect<3, double>({ spacing_dis_, -spacing_dis_, 0}),
         MTK::vect<3, double>({            0,  spacing_dis_, 0}),
         MTK::vect<3, double>({-spacing_dis_,  spacing_dis_, 0})
        });
    // clang-format on
    for (MTK::vect<3, double> &position : positions) { // 遍历九个点

      for (int j = 0; j < 12; j++) { // 在每个点旋转12次
        auto ps = position;
        Eigen::Vector3d axis(0.0, 0.0, 1.0); // 旋转轴
        double angle = -(M_PI * j / 6);      // 旋转角度
        Eigen::Quaterniond rot(Eigen::AngleAxisd(angle, axis.normalized()));
        std::vector<bool> effect_pts(initLocationCloud->size(), false);
        std::vector<Eigen::Matrix<double, 1, 6>> jacobians(
            initLocationCloud->size());
        std::vector<double> errors(initLocationCloud->size());
        for (int iter = 0; iter < max_iter_num_init_location; iter++) {
          // 开始迭代
          // 计算残差和雅可比
          for (int i = 0; i < initLocationCloud->size(); i++) {
            // 将点云转换到世界坐标系下
            PointType &point_body = initLocationCloud->points[i];
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                         ps);
            initLocationCloudWorld->points[i].x = p_global(0);
            initLocationCloudWorld->points[i].y = p_global(1);
            initLocationCloudWorld->points[i].z = p_global(2);
            initLocationCloudWorld->points[i].intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            PointVector points_near;
            ikdtree.Nearest_Search(initLocationCloudWorld->points[i],
                                   NUM_MATCH_POINTS, points_near,
                                   pointSearchSqDis);
            effect_pts[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                            : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                         : true;
            if (!effect_pts[i])
              continue;

            VF(4) pabcd;
            effect_pts[i] = false;
            if (!esti_plane(pabcd, points_near, 0.3f)) {
              continue;
            }
            float pd2 = pabcd(0) * initLocationCloudWorld->points[i].x +
                        pabcd(1) * initLocationCloudWorld->points[i].y +
                        pabcd(2) * initLocationCloudWorld->points[i].z +
                        pabcd(3);
            effect_pts[i] = true;
            Eigen::Matrix<double, 3, 3> hat_point_world;
            // clang-format off
              hat_point_world << 
                            0                      , -initLocationCloudWorld->points[i].z,  initLocationCloudWorld->points[i].y, 
                initLocationCloudWorld->points[i].y,              0                      , -initLocationCloudWorld->points[i].x, 
               -initLocationCloudWorld->points[i].y,  initLocationCloudWorld->points[i].x,              0                      ;
            // clang-format on

            Eigen::Matrix<double, 1, 6> J;
            Eigen::Matrix<double, 1, 3> a = {-pabcd(0), -pabcd(1), -pabcd(2)};
            Eigen::Matrix<double, 3, 3> b =
                rot.toRotationMatrix() * hat_point_world;
            J.block<1, 3>(0, 0) = a * b;
            J.block<1, 3>(0, 3) = -a;

            jacobians[i] = J;
            errors[i] = pd2;
          }
          // 更新
          Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
          Eigen::Matrix<double, 6, 1> err = Eigen::Matrix<double, 6, 1>::Zero();
          double total_res = 0;
          int effective_num = 0;
          for (int idx = 0; idx < initLocationCloud->size(); ++idx) {
            if (!effect_pts[idx]) {
              continue;
            }

            total_res += errors[idx] * errors[idx];
            effective_num++;

            H += jacobians[idx].transpose() * jacobians[idx];
            err -= jacobians[idx].transpose() * errors[idx];
          }
          if (effective_num < 5) {
            RCLCPP_WARN(this->get_logger(), "effective num too small");
            continue;
          }
          Eigen::Matrix<double, 6, 1> dx = H.inverse() * err;
          // 小车水平行驶默认翻滚和俯仰为零，加快收敛
          dx(0) = 0;
          dx(1) = 0;
          rot = rot * SO3::exp(dx.head<3>());
          ps += dx.tail<3>();
          if (debug_cloud_) {
            publish_map(this->pubLaserCloudMap_);
            sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*initLocationCloudWorld, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = this->get_clock()->now();
            laserCloudFullRes3.header.frame_id = "camera_init";
            this->pubLaserCloudEffect_->publish(laserCloudFullRes3);

            // 更新
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "iter " << iter << " total res: " << total_res
                        << ", eff: " << effective_num << ", mean res: "
                        << total_res / effective_num << ", dxn: " << dx.norm());
            // 睡眠一段时间便于肉眼观察
            this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.1));
          }

          if (dx.norm() < es_threshold_) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "converged, dx = " << dx.transpose());
            // 更新状态估计量
            auto state = kf.get_x();
            state.pos = ps;
            state.rot = rot;
            kf.change_x(state);
            need_init_location = false;
            return true;
          }
        }
      }
    }

    return false;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudFull_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr map_pub_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
  // 初始化定位

  double es_threshold_ = 0.005;        // 迭代收敛阈值
  double spacing_dis_ = 1;             // 点与点间隔距离
  int max_iter_num_init_location = 25; // 牛顿法迭代最大次数
  bool debug_cloud_ = true;

  bool effect_pub_en = false, map_pub_en = false;
  int effect_feat_num = 0, frame_num = 0;
  double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0,
                         aver_time_match = 0, aver_time_incre = 0,
                         aver_time_solve = 0, aver_time_const_H_time = 0;
  bool flg_EKF_converged, EKF_stop_flg = 0;
  double epsi[23] = {0.001};

  FILE *fp;
  ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, SigHandle);
  auto node = std::make_shared<LaserLocationNode>();
  rclcpp::spin(node);

  if (rclcpp::ok())
    rclcpp::shutdown();

  return 0;
}
