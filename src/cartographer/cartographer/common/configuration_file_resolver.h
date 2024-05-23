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

#ifndef CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_
#define CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_

#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

// A 'FileResolver' for the 'LuaParameterDictionary' that reads files from disk.
// It searches the 'configuration_files_directories' in order to find the
// requested filename. The last place searched is always the
// 'configuration_files/' directory installed with Cartographer. It contains
// reasonable configuration for the various Cartographer components which
// provide a good starting ground for new platforms.
class ConfigurationFileResolver : public FileResolver {
 public:

  // c++11: explicit关键字 的作用就是防止类构造函数的隐式自动转换
  // explicit关键字只能用于类内部的构造函数声明上，而不能用在类外部的函数定义(函数实现)上，它的作用是不能进行隐式转换；
  // explicit关键字作用于单个参数的构造函数，如果构造函数有多个参数，但是从第二个参数开始，如果各参数均有默认赋值，也可以应用explicit关键字
  // 当构造函数只有一个参数时，会进行自动隐式转换，当构造函数参数个数超过或等于两个时自动取消隐式转换，当只有一个必须输入的参数，其余的为有默认值的参数时使用explicit也起作用
  // 一般只将有单个参数的构造函数声明为explicit，而拷贝构造函数不要声明为explicit。
  // explicit关键字只能对用户自己定义的对象起作用，不对默认构造函数起作用。此关键只能够修饰构造函数。
  // 无参数的构造函数和多参数的构造函数总是显示调用，这种情况在构造函数前加explicit无意义。
  // 参考https://blog.csdn.net/fengbingchun/article/details/51168728
  // https://www.cnblogs.com/rednodel/p/9299251.html
  explicit ConfigurationFileResolver(
      const std::vector<std::string>& configuration_files_directories);

  std::string GetFullPathOrDie(const std::string& basename) override;
  std::string GetFileContentOrDie(const std::string& basename) override;

 private:
  // 两个路径 : 源文件，安装目录(如果有更多的目录可以自行添加)
  std::vector<std::string> configuration_files_directories_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_
