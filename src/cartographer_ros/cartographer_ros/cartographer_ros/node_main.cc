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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

/**
 * note: gflags是一套命令行参数解析工具
 * 参数定义
 * gflags主要支持的参数类型包括bool, int32, int64, uint64, double, string等，DEFINE_bool等在gflags.h中定义
 * 定义参数通过DEFINE_type宏实现, 该宏的三个参数含义分别为命令行参数名, 参数默认值, 以及参数的帮助信息
 * 当参数被定义后, 通过FLAGS_name就可访问到对应的参数
 */
// collect_metrics ：激活运行时度量的集合.如果激活, 可以通过ROS服务访问度量
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {// 无名命名空间，意味着命名空间中的标识符只能在本文件内访问，相当于给这个标识符加上了static，使得其可以作为内部连接

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.; // 常量表达式:值不会改变，编译器就能得到计算结果
  // 在tf2系统中，将包分为tf2和tf2_ros，前者用来进行坐标变换等具体操作，
  // tf2_ros则负责与ROS消息打交道，负责发布tf或订阅tf，即发布者和订阅者是在tf2_ros命名空间下的。
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)}; // tf变换的历史要保存10秒
  // 开启监听tf的独立线程
  tf2_ros::TransformListener tf(tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  // c++11: std::tie()函数可以将变量连接到一个给定的tuple上,生成一个元素类型全是引用的tuple, 解包tuple

  // 根据Lua配置文件中的内容, 为node_options, trajectory_options 赋值
  // FLAGS_configuration_directory,FLAGS_configuration_basename 是由命令行传入进来，经gflags解析出来的
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // MapBuilder类是完整的SLAM算法类,位于cartographer/mapping/map_builder.cc中
  // 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 
  // map_builder 为 std::unique_ptr<MapBuilderInterface>类型的 基类指针
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  
  // c++11: std::move 是将对象的状态或者所有权从一个对象转移到另一个对象, 
  // 只是转移, 没有内存的搬迁或者内存拷贝所以可以提高利用效率,改善性能..
  // 右值引用是用来支持转移语义的.转移语义可以将资源 ( 堆, 系统对象等 ) 从一个对象转移到另一个对象, 
  // 这样能够减少不必要的临时对象的创建、拷贝以及销毁, 能够大幅度提高 C++ 应用程序的性能.
  // 临时对象的维护 ( 创建和销毁 ) 对性能有严重影响.

  // Node类的初始化, 将ROS的topic传入SLAM, 也就是MapBuilder
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  // 如果加载了pbstream文件, 就执行这个函数(加载地图文件)
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // 使用默认topic 开始一条新的轨迹，一个轨迹就是一次slam过程，如果建好图了再进行定位，那就是2条轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();

  // 结束所有处于活动状态的轨迹
  node.FinishAllTrajectories();

  // 当所有的轨迹结束时, 再执行一次全局优化
  node.RunFinalOptimization();

  // 如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

/*
  argc：命令行参数个数(Argument Count)
  argv：命令行参数向量(Argument Vector):argv是一个字符串数组,其中第0个参数是程序的全名，后面的是命令行后面跟的用户输入的参数
*/
int main(int argc, char** argv) {

  // note: 初始化glog库
  google::InitGoogleLogging(argv[0]);
  
  // 使用gflags进行参数的初始化. 
  // 第三个参数表示是否释放argv以节约内存：
  //  如果为true,则该函数处理完成后,argv中只保留argv[0]程序名,argc会被设置为1;
  //  如果为false,则argv和argc都会被保留,但解析函数会调整argv中的顺序,
  // 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  // 通过launch 文件中的命令行参数指定，形如 ./main -files_path /usr/clude -file home.txt
  google::ParseCommandLineFlags(&argc, &argv, true);
  // 在argv中查找Flag标志并解析它们。重新排列argv以将标志放在首位，或者在remove_flags为true时完全删除它们。
  // 如果一个标志在命令行或标志文件中定义了多次，则使用最后一次定义。返回第一个非标志参数的索引(进入argv)

  /**
   * @brief glog里提供的CHECK系列的宏, 检测某个表达式是否为真
   * 检测expression如果不为真, 则打印后面的description和栈上的信息
   * 然后退出程序, 出错后的处理过程和FATAL比较像.
   */
  // 如果从命令行传入了这两个参数的话，表达式就为真，没有传入进来就为假，并退出程序
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // ros节点的初始化
  // 若launch文件中的节点名与此处不同，则该处节点名将被覆盖
  ::ros::init(argc, argv, "cartographer_node"); // :: 起全局作用域的作用
  // ::解析可参考 https://blog.csdn.net/qq1623803207/article/details/89398435

  // 一般不需要在自己的代码中显式调用
  // 但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start();
  // 实际上启动了节点的内部功能（启动线程、启动网络轮询和xmlrpc循环、连接到内部订阅（如/clock）、启动内部服务服务器等）
  // 通常不需要手动调用，因为如果节点尚未启动，则会在创建第一个NodeHandle时自动调用。
  // 如果您想防止最后一个NodeHandle超出范围而导致的自动关闭，请在创建任何NodeHandle之前调用此函数(例如，在init()之后立即调用)

  // 使用ROS_INFO进行glog消息的输出
  cartographer_ros::ScopedRosLogSink ros_log_sink;

  // 开始运行cartographer_ros
  cartographer_ros::Run();

  // 结束ROS相关的线程, 网络等
  ::ros::shutdown();
}
