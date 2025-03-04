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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
// 此接口同时用于2D和3D SLAM
// 将local slam 与 global slam 连接起来,实现了一个完整的slam
// 即用于初始姿势估计的 local SLAM, 
// 用于检测环路闭合的扫描匹配 以及 用于计算优化姿势估计的稀疏姿势图优化

/**
 * @brief TrajectoryBuilderInterface是个接口类,没有具体实现
 * GlobalTrajectoryBuilder类 与 CollatedTrajectoryBuilder类 都继承了 TrajectoryBuilderInterface
 * 并且都做了 AddSensorData() 的实现
 */
class TrajectoryBuilderInterface {
 public:
  struct InsertionResult {
    NodeId node_id; // 节点id
    std::shared_ptr<const TrajectoryNode::Data> constant_data;    // 前端匹配的结果，包含匹配后位姿，点云数据等
    std::vector<std::shared_ptr<const Submap>> insertion_submaps; // 前端活跃的两个子图
  };

  // c++11: std::function 通用多态函数封装器
  // std::function 的实例能存储、复制及调用任何可调用 (Callable) 目标: 
  // 如函数、 lambda表达式、 bind表达式或其他函数对象, 还有指向成员函数指针和指向数据成员指针.
  // 它也是对 C++ 中现有的可调用实体的一种类型安全的包裹（相对来说, 函数指针的调用不是类型安全的）

  // c++11: using in c++11: c++11 的using可以用于模板部分具体化

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, common::Time,
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;

  struct SensorId {
    // c++11: 限域枚举 enum class 
    enum class SensorType {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    SensorType type;  // 传感器类型
    std::string id;   // topic的名字

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  TrajectoryBuilderInterface() {}
  virtual ~TrajectoryBuilderInterface() {}

  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;

  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::LandmarkData& landmark_data) = 0;
  // Allows to directly add local SLAM results to the 'PoseGraph'. c++11 that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
