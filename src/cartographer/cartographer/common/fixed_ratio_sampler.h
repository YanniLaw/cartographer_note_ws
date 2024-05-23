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

#ifndef CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
#define CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_

#include <string>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

/*
FixedRatioSampler是频率固定的采样器类，目的是从数据流中均匀的按照固定频率采样数据
FixedRatioSampler不可拷贝,不可赋值.
成员函数提供2种操作:
Pulse()产生一个事件pulses,并且:如果计入采样samples，返回true
DebugString():以string形式输出采样率
*/
// Signals when a sample should be taken from a stream of data to select a
// uniformly distributed fraction of the data.
// 当应该从数据流中抽取样本以选择数据的均匀分布部分时发出信号
class FixedRatioSampler {
 public:
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();

  FixedRatioSampler(const FixedRatioSampler&) = delete;
  FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

  // Returns true if this pulse should result in an sample.
  // 在比例小于ratio_时返回true, ratio_设置为1时都返回true, 也就是说使用所有的数据
  bool Pulse();

  // Returns a debug string describing the current ratio of samples to pulses.
  std::string DebugString();

 private:
  // Sampling occurs if the proportion of samples to pulses drops below this
  // number.
  const double ratio_;

  int64 num_pulses_ = 0;  //产生的脉冲次数(也就是接收到消息的次数)
  int64 num_samples_ = 0; //记录的采样次数(也就是真正使用了消息的次数)
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
