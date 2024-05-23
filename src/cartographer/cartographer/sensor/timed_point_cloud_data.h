/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// 时间同步前的点云
struct TimedPointCloudData {
  common::Time time;        // 点云最后一个点的绝对时间 // origin　是lidar 在tracking frame 下的位置
  Eigen::Vector3f origin;   // 以tracking_frame_到雷达坐标系的坐标系变换(lidar到tracking的坐标变换)为原点,即 t_tracking_sensor
  TimedPointCloud ranges;   // 数据点(tracking_frame下)的集合, 每个数据点包含xyz与time, time是负的(相对时间)
  // origin其实就是激光雷达坐标系在tracking_frame中的位置
  // 'intensities' has to be same size as 'ranges', or empty.
  std::vector<float> intensities; // 空的
};

// 时间同步后的点云
struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;   // 带相对时间的单个数据点的坐标 xyz (tracking_frame下)
    float intensity;                    // 强度值
    size_t origin_index;                // 属于第几个origins的点(即属于哪个传感器的点云)
  };
  common::Time time;                    // 点云同步的时间
  std::vector<Eigen::Vector3f> origins; // 同步点云是由几个点云组成, 每个点云的原点(t_tracking_sensor)
  std::vector<RangeMeasurement> ranges; // 同步数据点的集合
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
