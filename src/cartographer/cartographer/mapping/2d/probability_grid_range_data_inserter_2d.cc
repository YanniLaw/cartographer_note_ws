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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.光线投射起点和终点的亚像素精度系数
constexpr int kSubpixelScale = 1000;

// 根据点云的bounding box, 看是否需要对地图进行扩张
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  // (Eigen::AlignedBox<float,2> float代表边界数据类型,2代表边界维度)
  // min()/max() 返回边界中最小/大的角点 
  // diagnol() 返回区域对角线的长度,即边界中最小和最大角点的距离
  // 找到点云的bounding_box
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>()); // t_local_gravity
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>()); // 拓展区域的边界,使得该区域包含输入的点
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  // 是否对地图进行扩张
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

/**
 * @brief 根据雷达点对栅格地图进行更新
 * 
 * @param[in] range_data local系下的点云数据
 * @param[in] hit_table 更新占用栅格时的查找表
 * @param[in] miss_table 更新空闲栅格时的查找表
 * @param[in] insert_free_space 
 * @param[in] probability_grid 栅格地图
 */
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  // 根据雷达数据调整地图范围
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits(); // 有可能在前一步发生了改变
  const double superscaled_resolution = limits.resolution() / kSubpixelScale; // 0.05mm 亚像素分辨率
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  // 雷达原点在地图中的像素坐标(更加精确的亚像素坐标), 作为画线的起始坐标
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    // 计算hit点在地图中的像素坐标(更加精确的亚像素坐标), 作为画线的终止点坐标
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    // 更新hit点的栅格值(查表)
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table); // 亚像素坐标要转换回像素坐标
  }
  
  // 如果不插入free空间就可以结束了
  if (!insert_free_space) { // 这个是对miss的网格(也就是射线穿过的网格)进行更新
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      // 从起点到end点之前, 更新miss点的栅格值
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      // 从起点到misses点之前, 更新miss点的栅格值
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

// 写入器的构造, 新建了2个查找表
/* options配置
  probability_grid_range_data_inserter = {
    insert_free_space = true, -- 如果为 false，则free的栅格不会改变 占用网格 中的概率
    hit_probability = 0.55,   -- hit(占用) 时的概率
    miss_probability = 0.49,  -- miss（穿过） 时的概率
    地图中栅格的更新状态有三种:hit(击中),miss(穿过),free(不更新)
  }
*/
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      // 生成更新占用栅格时的查找表 // param: hit_probability
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),    // 0.55
      // 生成更新空闲栅格时的查找表 // param: miss_probability
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {} // 0.49

/**
 * @brief 将点云写入栅格地图
 * 
 * @param[in] range_data 要写入地图的点云
 * @param[in] grid 栅格地图 ProbabilityGrid ---> Grid2D ---> GridInterface(虚基类)
 */
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // param: insert_free_space
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
