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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer_ros/trajectory_options.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  std::string map_frame;  // The ROS frame ID to use for publishing submaps, the parent frame of poses 
  double lookup_transform_timeout_sec;  // 查找tf时的超时时间
  double submap_publish_period_sec;     // 发布子图pose数据的时间间隔
  double pose_publish_period_sec;       // 发布pose数据的时间间隔
  double trajectory_publish_period_sec; // 发布轨迹marker数据的时间间隔
  bool publish_to_tf = true;            // Enable or disable providing of TF transforms
  bool publish_tracked_pose = false;    // Enable publishing of tracked pose as a `geometry_msgs/PoseStamped` to topic "tracked_pose"
  bool use_pose_extrapolator = true;    // 发布tf时是使用传感器融合出来的位姿还是前端计算出来的位姿
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
