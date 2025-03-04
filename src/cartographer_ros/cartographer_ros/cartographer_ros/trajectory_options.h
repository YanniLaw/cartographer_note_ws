/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer_ros {

// 一条轨迹的基础参数配置
struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  std::string tracking_frame;   // The ROS frame ID of the frame that is tracked by the SLAM algorithm.
  std::string published_frame;  // The ROS frame ID to use as the child frame for publishing poses
  // The frame between *published_frame* and *map_frame* to be used for publishing the (non-loop-closed) local SLAM result
  std::string odom_frame;       // Only used if *provide_odom_frame* is true
  bool provide_odom_frame; // If enabled, the local, non-loop-closed, continuous pose will be published as the *odom_frame* in the *map_frame*
  bool use_odometry;
  bool use_nav_sat;
  bool use_landmarks;
  bool publish_frame_projected_to_2d;
  int num_laser_scans;  // Number of laser scan topics to subscribe to.
  int num_multi_echo_laser_scans;
  /*Number of point clouds to split each received (multi-echo) laser scan into.
  Subdividing a scan makes it possible to unwarp scans acquired while the
  scanners are moving. There is a corresponding trajectory builder option to
  accumulate the subdivided scans into a point cloud that will be used for scan
  matching.*/
  int num_subdivisions_per_laser_scan;
  int num_point_clouds;
  double rangefinder_sampling_ratio;
  double odometry_sampling_ratio;
  double fixed_frame_pose_sampling_ratio;
  double imu_sampling_ratio;
  double landmarks_sampling_ratio;
};

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
