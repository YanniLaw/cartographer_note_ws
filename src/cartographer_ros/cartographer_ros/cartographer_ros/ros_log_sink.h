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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H

#include <ctime>

#include "glog/logging.h"

namespace cartographer_ros {

// Makes Google logging use ROS logging for output while an instance of this
// class exists.
/**
 * @brief 自定义的输出日志的方式: 使用ROS_INFO进行glog消息的输出
 */
class ScopedRosLogSink : public ::google::LogSink {
 public:
  ScopedRosLogSink();
  ~ScopedRosLogSink() override;

  void send(::google::LogSeverity severity, const char* filename,
            const char* base_filename, int line, const struct std::tm* tm_time,
            const char* message, size_t message_len) override;

  void WaitTillSent() override;
  // override: 1.在函数比较多的情况下可以提示读者某个函数重写了基类虚函数（表示这个虚函数是从基类继承，不是派生类自己定义的）；
  // 2.强制编译器检查某个函数是否重写基类虚函数，如果没有则报错。

 private:
  bool will_die_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H