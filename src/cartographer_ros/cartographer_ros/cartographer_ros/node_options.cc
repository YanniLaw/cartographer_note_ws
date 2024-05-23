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

#include "cartographer_ros/node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "glog/logging.h"

namespace cartographer_ros {

/**
 * @brief 读取lua文件内容, 将lua文件的内容赋值给NodeOptions
 * 
 * @param lua_parameter_dictionary lua字典
 * @return NodeOptions 
 */
NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
          
  NodeOptions options;

  // 根据lua字典中的参数, 生成protobuf的序列化数据结构 proto::MapBuilderOptions
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());

  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
    options.publish_to_tf =
        lua_parameter_dictionary->GetBool("publish_to_tf");
  }
  if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
    options.publish_tracked_pose =
        lua_parameter_dictionary->GetBool("publish_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
    options.use_pose_extrapolator =
        lua_parameter_dictionary->GetBool("use_pose_extrapolator");
  }
  return options;
}

/**
 * @brief 加载lua配置文件中的参数
 * 
 * @param[in] configuration_directory 配置文件所在目录
 * @param[in] configuration_basename 配置文件的名字
 * @return std::tuple<NodeOptions, TrajectoryOptions> 返回节点的配置与轨迹的配置
 */
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  // 获取配置文件所在的目录
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory}); // std::string -> std::vector<std::string>
        
  // 读取配置文件内容到code中,现在code是配置文件configuration_directory/configuration_basename里面的所有内容了
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);

  
  /* 在c++中，一个值要么是右值，要么是左值，左值是指表达式结束后依然存在的持久化对象，右值是指表达式结束时就不再存在的临时对象。
   所有的具名变量或者对象都是左值，而右值不具名。
   std::move并不能移动任何东西，它唯一的功能是将一个左值强制转化为右值引用，继而可以通过右值引用使用该值，以用于移动语义
   1.C++ 标准库使用比如vector::push_back 等这类函数时,会对参数的对象进行复制,连数据也会复制.这就会造成对象内存的额外创建, 本来原意是想把参数push_back进去就行了,通过std::move，可以避免不必要的拷贝操作。
   2.std::move是将对象的状态或者所有权从一个对象转移到另一个对象，只是转移，没有内存的搬迁或者内存拷贝所以可以提高利用效率,改善性能。
    可参考:https://blog.csdn.net/p942005405/article/details/84644069
    https://zhuanlan.zhihu.com/p/94588204
  */
  // 根据给定的字符串, 生成一个lua字典
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  // 创建元组tuple,元组定义了一个有固定数目元素的容器, 其中的每个元素类型都可以不相同
  // 将配置文件的内容填充进NodeOptions与TrajectoryOptions, 并返回
  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace cartographer_ros
