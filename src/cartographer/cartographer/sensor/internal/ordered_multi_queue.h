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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "cartographer/common/internal/blocking_queue.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace sensor {

struct QueueKey {
  int trajectory_id;      // 轨迹id
  std::string sensor_id;  // topic名字

  // 重载小于运算符, map根据这个规则对QueueKey进行排序
  // 以tuple规则比较2者, tuple定义了<运算符, 逐个元素进行比较
  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
// 维护排序后的传感器数据的多个队列, 并按合并排序的顺序进行调度
// 它将等待为每个未完成的队列查看至少一个值, 然后再在所有队列中分派下一个按时间排序的值.

// This class is thread-compatible. 此类是线程兼容的

/*
  OrderedMultiQueue，用于管理多个有序的传感器数据，
  是有序的多队列类,每个队列有一个key,并且有一个自定义排序函数
  queues_的形式为：
  key1:Queue
  key2：Queue
  key3：Queue

*/

class OrderedMultiQueue {
 public:
  // note: OrderedMultiQueue::Callback 1个参数
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue();

  // c++11: 移动构造函数, 只在使用的时候编译器才会自动生成
  // 这里是显示指定让编译器生成一个默认的移动构造函数
  OrderedMultiQueue(OrderedMultiQueue&& queue) = default;

  ~OrderedMultiQueue();

  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  void AddQueue(const QueueKey& queue_key, Callback callback);

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  void MarkQueueAsFinished(const QueueKey& queue_key);

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  void Flush();

  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;   // 存储传感器数据的队列(阻塞队列:构造生产者/消费者模型)
    Callback callback;                                    // 本数据队列对应的回调函数
    bool finished = false;                                // 这个queue是否finished(其实就是还用不用这个传感器数据了)
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.上一次数据分发的时间
  // 根据不同传感器数据到来的时间先后顺序进行的数据分发
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_; // 每条轨迹保存一个开始时间
  // QueueKey 为{轨迹id，传感器topic}
  std::map<QueueKey, Queue> queues_;   // 多个传感器数据队列(也包括多个轨迹)
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
