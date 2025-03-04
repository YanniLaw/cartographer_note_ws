-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", -- 进行slam的坐标系，如果有imu的话为imu的坐标系，没有的话就是base_link 或者 base_footprint
  published_frame = "base_link", -- carto发布的坐标系
  odom_frame = "odom",
  --if false: map_frame -> published_frame(base_link), if true: map -> odom -> published_frame(base_link)
  provide_odom_frame = true, -- 是否发布里程计坐标系，适用于机器人本身没有odom坐标系的情况
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 1,
  num_subdivisions_per_laser_scan = 10, -- 将雷达一帧的数据拆分成几次发出来，对于普通雷达的驱动包来说，此处应为1
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1., -- 接下来的5个参数为5种观测的权重比
  odometry_sampling_ratio = 1., -- 如odom的数据非常不准，可以设置为0.3以减小odom对整体优化的影响
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
