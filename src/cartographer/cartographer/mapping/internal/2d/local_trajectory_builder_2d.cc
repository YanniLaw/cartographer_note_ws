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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

// note: 所有传入的数据都已经变换到了tracking坐标系下!!!!
/**
 * @brief 构造函数
 * 参数位于 cartographer/configuration_files/trajectory_builder_2d.lua文件中
 * @param[in] options 参数位于cartographer/configuration_files/trajectory_builder_2d.lua
 * @param[in] expected_range_sensor_ids 所有range类型的话题
 */
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

/**
 * @brief 先进行点云的旋转与z方向的滤波, 然后再进行体素滤波减少数据量
 * 
 * @param[in] transform_to_gravity_aligned_frame 将点云变换到原点处, 且姿态为0的坐标变换 t_local_tracking
 * @param[in] range_data 传入的点云(in local frame)
 * @return sensor::RangeData 处理后的点云 拷贝
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  // Step: 5 将原点位于机器人当前位姿处的点云 转成 原点位于local坐标系原点处的点云, 再进行z轴上的过滤 // 错误说法
  /// 应该是将位于local坐标系下的点云变换到GravityAligned坐标系下(与tracking frame位置相同，但Z轴与重力方向反向，相当于转回到tracking frame下进行操作)
  // cropped.origin = transform_to_gravity_aligned_frame*range_data.origin
  const sensor::RangeData cropped = // cropped.origin = {0,0,0}
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z()); // param: min_z:-1.8m, max_z: 2m
  // Step: 6 对点云进行体素滤波
  return sensor::RangeData{
      cropped.origin,// [0,0,0]^T
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()), // param: voxel_filter_size
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

/**
 * @brief 进行扫描匹配
 * 
 * @param[in] time 点云的时间
 * @param[in] pose_prediction 先验位姿(x,y,yaw) T_local_gravity
 * @param[in] filtered_gravity_aligned_point_cloud 匹配用的点云(位于重力对齐坐标系下)
 * @return std::unique_ptr<transform::Rigid2d> 匹配后的二维位姿
 */
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  if (active_submaps_.submaps().empty()) { // 此时还未创建子图(初始化时)
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 使用active_submaps_的第一个子图进行匹配
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  // 根据参数决定是否 使用correlative_scan_matching对先验位姿进行校准
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose); // 粗匹配
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  // 使用ceres进行扫描匹配
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary); // 精匹配
  // 一些度量
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 返回ceres计算后的位姿
  return pose_observation;
}

/**
 * @brief 处理点云数据, 进行扫描匹配, 将点云写成地图
 * 从这里开始，激光点云都转到了tracking坐标系下，那么这其实可以说他的原点其实就是tracking frame原点
 * @param[in] sensor_id 点云数据对应的话题名称
 * @param[in] unsynchronized_data 传入的点云数据(未经时间同步,已经转到tracking frame下)
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 匹配后的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  
  // Step: 1 进行多个距离传感器点云数据的时间同步, 坐标也会进行同步(点云的坐标位于tracking_frame的)
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) { // 正在进行同步,等待另外的距离传感器数据....
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  // 同步后的时间(依然是同步点云最后一个点的绝对时间)
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  // 如果不用imu, 就在雷达这初始化位姿推测器,然后再根据扫描匹配的结果更新位姿
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }
  // 总之 都要首先初始化位姿推测器, 才可以进行到下一步
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);

  // 计算第一个点的时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  // 只有在extrapolator_初始化时, GetLastPoseTime()是common::Time::min()
  // 将time_first_point与位姿估计器的时间对比来判定位姿估计器是否完成了初始化操作
  // tip: 如果用lidar信息初始化位姿估计器，那么第一个点的时间肯定是小于估计器时间的,就不进行下面的操作了
  // 第一个点的绝对时间小于外推器(上一次加入前端匹配位姿后的Pose)的时间，那么就不用这一帧数据了，因为此时无法利用外推器数据进行畸变矫正
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  std::vector<transform::Rigid3f> range_data_poses; // 每个点带一个位姿，用于去掉畸变
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;

  // 预测得到每一个时间点的位姿(一般是计算激光雷达发射原点位姿:在这里由于转换到了tracking frame下,所以计算的是tracking frame原点)
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time); // 每个点的绝对时间
    // 如果该时间比上次预测位姿的时间还要早,说明这个点的时间戳往回走了, 就报错 /// 应该是更新吧这里
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      // 一个循环只报一次错
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    
    // Step: 2 预测出 每个激光点(已经转到了tracking frame下)对应的发射原点(tracking frame坐标原点) 在 local slam 坐标系下的位姿
    // 注意，这里为什么计算的是tracking frame坐标原点在 local slam 坐标系下的位姿，这是因为现在所有的激光点云数据已经转到了统一的tracking frame下面来表示
    // 即synchronized_data.ranges这里面的值其实是相对于tracking frame坐标原点的值
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());// 在上一次优化的位姿上进行外推
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 对每个数据点进行处理
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // 获取当前激光点在　tracking frame 下点的坐标 hit_in_tracking
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    // 将点云坐标系的原点转到 local slam 坐标系下
    // 其实就是求出激光雷达坐标系原点在local slam下的坐标 
    // t_local_lidar = R_local_tracking * t_tracking_lidar + t_local_tracking
    const Eigen::Vector3f origin_in_local = // lidar坐标系原点在local坐标系下的坐标
        range_data_poses[i] * // T_local_tracking
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index); // t_tracking_lidar(lidar坐标系原点在tracking下的位置)
    
    // Step: 3 运动畸变的去除, 将点相对于tracking_frame的hit坐标 转成 local坐标系下的坐标
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit); // 激光点在local坐标系下的坐标
    
    // 计算这个激光点去畸变之后(从发射点到hit点)的距离(这里将每个点都转到了local坐标系下面进行计算)
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm(); // 二范数,现在这个距离就是在local frame 下的距离度量了
    
    // param: min_range max_range
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        // 在这里可以看到, returns里保存的是local slam下的去畸变之后的点的坐标
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // Step: 4 超过max_range时的处理: 用一个距离进行替代, 并放入misses里
        hit_in_local.position =
            origin_in_local +
            // param: missing_data_ray_length, 是个比例, 不是距离
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  } // end for

  // 有一帧有效的数据了
  ++num_accumulated_;

  // param: num_accumulated_range_data 几帧有效的点云数据进行一次扫描匹配
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 计算2次有效点云数据的的时间差
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;

    // 重置变量
    num_accumulated_ = 0;

    // 获取机器人当前姿态(三维) tracking frame in local frame，R_local_tracking
    // 有用加速度数据进行重力对齐等操作 gravity_alignment R:预测姿态　t: 0
    // 这里　gravity_alignment其实是与重力向量的变化的四元数，所以才叫重力对齐(roll,pitch是准的，yaw不准)
    // 预测到time时刻的imu_tracker_姿态(其实就是相对于固定重力向量姿态的变化)，imu_tracker_时间递进到time时刻
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));

    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    // 以最后一个点的时间戳外推出的坐标为这帧(或者几帧)数据(已经转到了tracking frame的点云)的原点(即tracking frame原点)在local frame下的坐标 
    // tracking frame in local slam frame, t_local_tracking，去畸变时对每个点做了位姿估计
    accumulated_range_data_.origin = range_data_poses.back().translation(); //Eigen::Vector3f 
    // 变换矩阵求逆            
    // T^-1 = | R^T    -R^T*t |
    //        | 0       1     |

    // gravity_alignment.cast<float>() * range_data_poses.back().inverse() = 
    // 
    // |  R_local_tracking  0 | * | R_local_tracking  t_local_tracking  |^-1 = | Identity  t_tracking_local |
    // |  0^T               1 |   |     0^T                 1           |      |  0^T           1           |
    return AddAccumulatedRangeData(
        time,
        // 将点云变换到tracking坐标系原点处, 且姿态为0 ，即重力对齐坐标系(根据重力加速度的方向来估计激光扫描数据在水平面上的投影)
        TransformToGravityAlignedFrameAndFilter(
            // 2d建图时这个乘积的旋转部分其实是Identity ，平移部分t = -t_local_tracking = t_tracking_local
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_), // accumulated_range_data_的点云数据存的都是相对与local坐标系下的坐标
        gravity_alignment, sensor_duration);
  }

  return nullptr;
}

/**
 * @brief 进行扫描匹配, 将点云写入地图
 * 
 * @param[in] time 点云的时间戳
 * @param[in] gravity_aligned_range_data 原点位于tracking坐标系原点处的点云(姿态为0) ,即重力对齐坐标系下的点云
 * @param[in] gravity_alignment 机器人当前姿态(3d),平移部分为0
 * @param[in] sensor_duration 2帧点云数据的时间差
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data, // gravity_aligned_range_data.origin = {0,0,0}
    const transform::Rigid3d& gravity_alignment, // R_local_tracking
    const absl::optional<common::Duration>& sensor_duration) {
  // 如果处理完点云之后数据为空, 就报错. 使用单线雷达时不要设置min_z
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 进行位姿的预测, 先验位姿(在优化后的位姿的基础上外推的)
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time); /// 按理说应该等于去畸变时最后一个点的位姿
  // 将三维位姿先旋转到姿态为0, 再取xy坐标(将三维位姿转成二维位姿)
  // 这里其实是求的重力对齐坐标系相对于local的位姿
  // | R_local_tracking  t_local_tracking | * | R_local_tracking  0 |^-1 = | Identity  t_local_tracking |
  // |    0^T                   1         |   |     0^T           1 |      |   0^T           1          |
  // 为什么要转2维???这是因为scan match最终优化的只有二维的x,y,yaw三个变量，而且现在点云也已经转到了重力对齐坐标系下了
  // pose_prediction 只保留了x,y,yaw三个变量(TODO: 这里的yaw是什么东西???)
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // Step: 7 对 returns点云 进行自适应体素滤波，返回的点云的数据类型是PointCloud
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 扫描匹配, 进行点云与submap的匹配,返回匹配优化后的位姿
  // 注意这里是　transform::Rigid2d ,其实就是优化了x,y,yaw三个变量
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);

  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  // 将二维坐标旋转回之前的姿态，主要是把优化后的偏航角加到3d上
  // 这里为什么只需要叠加优化后的偏航角就行了，其实是因为imu的固有原因，只用加速度计跟陀螺仪的话在roll,pitch的估计上是很准确的
  // 所以放到yaw角上他的估计偏差就比较明显了，虽然imu_tracker有用互补滤波去补偿yaw角，但是每个轴其实他的偏差都是不一样的
  // 如果要准确的来估计yaw，那么还需要另外一个观测，这里就是把激光点云的匹配当成了一个观测(虽然这种观测也是有偏差的)，但是后面可以通过回环可以抑制
  // (其实从数据融合的角度来看roll,pitch都有各自的一个观测[即加速度计的值，本身加速度计与陀螺仪就可以做融合解算姿态，现有的六轴模块就是这样做的]，但是yaw没有观测)
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 校准位姿估计器(tracking frame 相对于local frame的位姿,z轴位置为0)
  extrapolator_->AddPose(time, pose_estimate);

  // Step: 8 将 原点位于重力对齐坐标系原点处的点云 转换到local坐标系下(用优化后的位姿进行变换，本质上只涉及x,,y,yaw三个度量转换)
  // range_data_in_local.origin = t_local_gravity 其实还是一个水平的点云
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  
  // 将校正后的雷达数据写入submap
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

  // 计算耗时
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  // 计算cpu耗时
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;

  // 返回结果
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

/**
 * @brief 将处理后的雷达数据写入submap
 * 
 * @param[in] time 点云的时间
 * @param[in] range_data_in_local 校正后的点云(位于local坐标系下，经过了scan match 优化),负责插入子图中
 * @param[in] filtered_gravity_aligned_point_cloud 自适应体素滤波后的点云(位于重力对齐坐标系下)
 * @param[in] pose_estimate 扫描匹配(优化)后的三维位姿(相对local slam)
 * @param[in] gravity_alignment 机器人的姿态 
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 如果移动距离过小, 或者时间过短, 不进行地图的更新(第一帧数据直接返回,每成功调用一次便更新其值)
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 将点云数据写入到submap中 (insertion_submaps为返回的两个(最多)子图信息)
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  // 生成InsertionResult格式的数据进行返回
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,  // 这里存的是体素滤波后的点云(已经去了畸变)
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}), // 最后优化后的最佳位姿
      std::move(insertion_submaps)});
}

// 将由数据分发来的IMU数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

// 将里程计数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) { // 要先初始化再加入里程计数据
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 如果Extrapolator没有初始化就进行初始化
// 这里的time是传感器经过数据分发后的时间
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 如果已经初始化过了就直接返回
  if (extrapolator_ != nullptr) {
    return;
  }

  // 注意 use_imu_based为true就会报错 位于cartographer/configuration_files/trajectory_builder_2d.lua
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.

  // 初始化位姿推测器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()), // 0.001s
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant()); // 10
  // 添加初始位姿
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
