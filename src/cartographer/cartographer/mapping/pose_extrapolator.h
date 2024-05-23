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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

/*
  PoseExtrapolator的主要功能就是利用IMU ,Odometry惯导加local激光SLAM得出的Pose进行融合得到姿态(平移, 旋转), 
  因为IMU, Odometry的速率比激光有高, 在一定时间段内可能没有Pose数据,则通过IMU, Odometry推断,
  同时因为他们存在累积误差, 在有Pose的时候则对他们进行一些更新和初始化消除累积误差. 
  同时,在估计旋转的时候, 可以在没有IMU的时候可以利用之前的Pose 或者 Odometry估算出的速度进行模拟在一定程度上可以保持位姿数据的稳定性.
*/

/*
  位姿估计器解决的是在实现前端scan-match之前的预测值，即激光新的一帧匹配前获取当前预测定位信息。
  同时假设传感器数据之间，slam本体以恒定线速度和恒定角速度移动。根据上次的scan-match的准确位置后进行积分，获取预测位置和姿态。
  
  其中估计的恒定速度主要两种方式组成：
  1、采用两次scan-match位置进行微分获得；
  2、另一种通过里程计进行微分获得；
  由于里程计的更新速度更快，故里程计可计算最新结果时，优先使用；
  
  估计的角速度则有三种方式组成：
  1.采用两次scan-match的航向进行微分获得，频率最低；
  2.采用里程计进行微分；
  3.采用IMU直接获取，优先级最高；

  cartographer针对imu和odometer是否使用可进行配置，根据配置信息和时间序列进行选择具体方式，
  最后根据估计的线速度和角速度进行实时积分，获取预测估计位置。
  最基本的配置odometry和imu均无，则估计器实际上仅是根据历史位置进行推算出线速度和角速度，进行估计位置。
  说到这里也可以通过后端优化node数设置为0,单独用odom、imu来检测前端的效果
*/

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
// 保持poses一定持续时间, 以估计线速度和角速度(特别注意：估计出来的线速度都是位于local坐标系下的,角速度都是位于tracking下)
// 使用速度预测运动. 使用IMU和/或里程计数据（如果有）来改善预测
class PoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // c++11: override 关键字告诉编译器, 该函数应覆盖基类中的函数.
  // 如果该函数实际上没有覆盖任何函数, 则会导致编译器错误
  // 如果没加这个关键字 也没什么严重的error 只是少了编译器检查的安全性

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  // 保存一定时间内的pose
  const common::Duration pose_queue_duration_;  // 两次预测估计最小时间差
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  // 线速度有2种计算途径,一种是根据pose计算，另一种是根据odom计算
  // 相邻pose计算出的线速度和角速度
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  // deque 双端队列　能够在首尾进行快速地插入和删除操作
  std::deque<sensor::ImuData> imu_data_; // 原始imu数据队列，一般仅保留两个或最新预测之后的所有序列

  // c++11: std::unique_ptr 是独享被管理对象指针所有权的智能指针
  // 它无法复制到其他 unique_ptr, 也无法通过值传递到函数,也无法用于需要副本的任何标准模板库 (STL) 算法
  // 只能通过 std::move() 来移动unique_ptr
  // std::make_unique 是 C++14 才有的特性

  // imu_tracker_: 直接使用IMU数据进行，只进行一次初始化
  // odometry_imu_tracker_: 表示AddPose到AddOdometryData之间（即 最新优化的位姿时刻 到 最新odom数据时刻 之间的时间段）的姿态变化
  // extrapolation_imu_tracker_: 表示AddPose到ExtrapolatePose之间（即 最新优化的位姿时刻 到 最新估计位姿时刻 之间的时间段）的姿态变化
  std::unique_ptr<ImuTracker> imu_tracker_;               // 保存与预测当前姿态(全局的航向推算器)
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;      // 用于计算里程计的姿态的ImuTracker(临时的航向推算器)
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_; // 用于预测姿态的ImuTracker
  TimedPose cached_extrapolated_pose_;    // 推算的新的位姿缓存

  std::deque<sensor::OdometryData> odometry_data_; // 原始odom数据队列，一般仅保留两个或最新预测之后的所有序列
  // odom数据计算出来的线速度和角速度
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
