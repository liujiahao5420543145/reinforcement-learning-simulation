// Modified from MuJoCo 3.17.0 simulate/main.cc: add inverted pendulum PD control and terminal telemetry.
// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>  // 引入系统调用错误码
#include <chrono>  // 引入单调时钟和时间间隔类型
#include <cstdint>  // 引入固定宽度整数类型
#include <cstdio>  // 引入格式化终端输出及缓冲刷新函数
#include <cstdlib>  // 引入进程退出等标准工具
#include <cstring>  // 引入内存清零和错误字符串函数
#include <iostream>  // 引入标准错误输出流
#include <memory>  // 引入智能指针以管理动态资源
#include <cmath>  // 引入绝对值和有限数检查等数学函数
#include <mutex>  // 引入互斥锁和自动加锁工具
#include <new>  // 引入不抛出异常的内存分配标记
#include <string>  // 引入字符串存储和路径处理功能
#include <thread>  // 引入后台物理线程及线程休眠工具

#include <mujoco/mujoco.h>  // 引入 MuJoCo 模型、状态、动力学及可视化 API
#include "glfw_adapter.h"  // 引入官方查看器的 GLFW 窗口适配器
#include "simulate.h"  // 引入 MuJoCo 官方查看器类与界面状态
#include "array_safety.h"  // 引入带数组容量检查的字符串工具
#include "simulation.h"  // 引入仿真启动接口声明
#include "pendulum_control.h"  // 引入 PID 参数与控制器接口声明
#include "telemetry.h"  // 引入终端遥测输出接口声明
#include "pid_dashboard.h"  // 引入调参面板与响应采样接口声明

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"  // 定义可执行文件旁用于存放 MuJoCo 插件的目录名

extern "C" {  // 以 C 语言链接约定包含平台系统接口
#if defined(_WIN32) || defined(__CYGWIN__)  // 仅在 Windows 或 Cygwin 平台编译下面的分支
  #include <windows.h>  // 引入 Windows 可执行文件路径查询 API
#else  // 切换到当前预处理条件不成立时使用的平台实现
  #if defined(__APPLE__)  // 仅在 macOS 平台编译下面的分支
    #include <mach-o/dyld.h>  // 引入 macOS 可执行文件路径查询 API
  #endif  // 结束当前平台条件编译分支
  #include <errno.h>  // 引入 POSIX 错误码定义
  #include <unistd.h>  // 引入 POSIX 符号链接读取 API
#endif  // 结束当前平台条件编译分支
}  // 结束当前函数、命名空间或控制分支的作用域

namespace {  // 进入匿名命名空间，使内部工具及状态仅在本文件可见
namespace mj  = ::mujoco;  // 将官方 MuJoCo 命名空间简写为 mj
namespace mju = ::mujoco::sample_util;  // 将官方样例的安全字符串工具命名空间简写为 mju

// 实时同步参数：允许的时间偏差、每帧用于物理推进的时间比例、错误缓冲大小。
const double syncMisalign       = 0.1;  // maximum mis-alignment before re-sync (simulation seconds)  // 实际时间与仿真时间偏差超过 0.1 秒时重新同步
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation  // 每帧最多使用约 70% 的刷新周期推进物理仿真
const int    kErrorLength       = 1024;  // load error string length  // 模型加载错误信息缓冲区容量为 1024 字节

// 当前模型及状态：物理线程拥有其生命周期，与界面交互时通过 sim.mtx 保护。
mjModel* m = nullptr;  // 保存当前编译后的模型，尚未加载时为空指针
mjData*  d = nullptr;  // 保存当前模型的动态状态，尚未创建时为空指针

using Seconds = std::chrono::duration<double>;  // 定义以秒为单位的双精度时间间隔类型


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
// 获取可执行文件目录，用于查找同级的插件目录；失败时返回空字符串。
std::string getExecutableDir() {  // 获取当前可执行文件所在目录，用于定位插件
#if defined(_WIN32) || defined(__CYGWIN__)  // 仅在 Windows 或 Cygwin 平台编译下面的分支
  constexpr char kPathSep = '\\';  // Windows 路径使用反斜杠作为目录分隔符
  std::string    realpath = [&]() -> std::string {  // 通过立即调用的 Lambda 查询并返回可执行文件完整路径
    std::unique_ptr<char[]> realpath(nullptr);  // 用智能指针管理可按需扩容的路径字符缓冲区
    DWORD                   buf_size = 128;  // Windows 路径缓冲区初始容量为 128 个字符
    bool                    success  = false;  // 记录是否成功取得完整路径
    while (!success) {  // 持续尝试读取路径，直到成功或遇到不可恢复错误
      realpath.reset(new (std::nothrow) char[buf_size]);  // 按当前容量分配路径缓冲区，失败时返回空指针而非抛出异常
      if (!realpath) {  // 检查路径缓冲区是否分配失败
        std::cerr << "cannot allocate memory to store executable path\n";  // 向标准错误流报告路径内存分配失败
        return "";  // 返回空路径，表示没有可用的可执行文件目录
      }  // 结束当前函数、命名空间或控制分支的作用域

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);  // 调用 Windows API 获取当前可执行文件完整路径
      if (written < buf_size) {  // 读取结果长度小于缓冲区容量时按当前实现进入成功分支
        success = true;  // 标记完整路径已经取得，结束重试循环
      } else if (written == buf_size) {  // 返回长度等于容量时，说明需要扩大路径缓冲区
        // realpath is too small, grow and retry
        buf_size *= 2;  // 将路径缓冲区容量扩大一倍，下一轮重新读取
      } else {  // 当前条件不满足时，进入另一处理分支
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";  // 报告 Windows 路径查询失败及系统错误码
        return "";  // 返回空路径，表示没有可用的可执行文件目录
      }  // 结束当前函数、命名空间或控制分支的作用域
    }  // 结束当前函数、命名空间或控制分支的作用域
    return realpath.get();  // 将路径字符缓冲区转换为返回的字符串
  }();  // 立即执行前面定义的 Lambda，并用其结果初始化字符串
#else  // 切换到当前预处理条件不成立时使用的平台实现
  constexpr char kPathSep = '/';  // POSIX 平台路径使用正斜杠作为目录分隔符
  #if defined(__APPLE__)  // 仅在 macOS 平台编译下面的分支
  std::unique_ptr<char[]> buf(nullptr);  // 用智能指针管理 macOS 路径查询缓冲区
  {  // 建立局部作用域，限制临时变量或锁的生命周期
    std::uint32_t buf_size = 0;  // 先将容量设为零以查询实际所需路径长度
    _NSGetExecutablePath(nullptr, &buf_size);  // 向 macOS 查询可执行文件路径所需的缓冲区大小
    buf.reset(new char[buf_size]);  // 按查询到的容量分配 macOS 路径缓冲区
    if (!buf) {  // 检查路径缓冲区是否为空
      std::cerr << "cannot allocate memory to store executable path\n";  // 向标准错误流报告路径内存分配失败
      return "";  // 返回空路径，表示没有可用的可执行文件目录
    }  // 结束当前函数、命名空间或控制分支的作用域
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {  // 再次查询可执行文件路径，并判断 API 是否返回错误
      std::cerr << "unexpected error from _NSGetExecutablePath\n";  // 报告 macOS 可执行文件路径查询异常
    }  // 结束当前函数、命名空间或控制分支的作用域
  }  // 结束当前函数、命名空间或控制分支的作用域
  const char* path = buf.get();  // 将 macOS 查询结果作为后续路径解析的输入
  #else  // 切换到当前预处理条件不成立时使用的平台实现
  const char* path = "/proc/self/exe";  // Linux 通过 proc 文件系统中的链接定位当前可执行文件
  #endif  // 结束当前平台条件编译分支
  std::string realpath = [&]() -> std::string {  // 通过立即调用的 Lambda 读取可执行文件符号链接目标
    std::unique_ptr<char[]> realpath(nullptr);  // 用智能指针管理可按需扩容的路径字符缓冲区
    std::uint32_t           buf_size = 128;  // POSIX 路径缓冲区初始容量为 128 字节

    bool success = false;  // 初始化路径读取成功标记
    while (!success) {  // 持续尝试读取路径，直到成功或遇到不可恢复错误
      realpath.reset(new (std::nothrow) char[buf_size]);  // 按当前容量分配路径缓冲区，失败时返回空指针而非抛出异常
      if (!realpath) {  // 检查路径缓冲区是否分配失败
        std::cerr << "cannot allocate memory to store executable path\n";  // 向标准错误流报告路径内存分配失败
        return "";  // 返回空路径，表示没有可用的可执行文件目录
      }  // 结束当前函数、命名空间或控制分支的作用域

      std::size_t written = readlink(path, realpath.get(), buf_size);  // 读取可执行文件链接指向的实际路径及其长度
      if (written < buf_size) {  // 读取结果长度小于缓冲区容量时按当前实现进入成功分支
        realpath.get()[written] = '\0';  // 在已读路径末尾补上字符串终止符

        success = true;  // 标记完整路径已经取得，结束重试循环
      } else if (written == -1) {  // 处理符号链接读取失败的分支
        if (errno == EINVAL) {  // 若目标不是符号链接，则可以直接使用原始路径
          // path is already not a symlink, just use it
          return path;  // 将输入路径作为已解析路径返回
        }  // 结束当前函数、命名空间或控制分支的作用域

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';  // 报告路径解析失败时的系统错误说明
        return "";  // 返回空路径，表示没有可用的可执行文件目录
      } else {  // 当前条件不满足时，进入另一处理分支
        // realpath is too small, grow and retry
        buf_size *= 2;  // 将路径缓冲区容量扩大一倍，下一轮重新读取
      }  // 结束当前函数、命名空间或控制分支的作用域
    }  // 结束当前函数、命名空间或控制分支的作用域
    return realpath.get();  // 将路径字符缓冲区转换为返回的字符串
  }();  // 立即执行前面定义的 Lambda，并用其结果初始化字符串
#endif  // 结束当前平台条件编译分支

  if (realpath.empty()) { return ""; }  // 路径解析结果为空时直接返回空目录

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {  // 从路径末尾向前寻找最后一个目录分隔符
    if (realpath.c_str()[i] == kPathSep) { return realpath.substr(0, i); }  // 找到分隔符后截取其前面的目录部分并返回
  }  // 结束当前函数、命名空间或控制分支的作用域

  // don't scan through the entire file system's root
  return "";  // 返回空路径，表示没有可用的可执行文件目录
}  // 结束当前函数、命名空间或控制分支的作用域


// scan for libraries in the plugin directory to load additional plugins
// 枚举已链接插件，并从可执行文件旁的 mujoco_plugin 目录加载扩展库。
void scanPluginLibraries() {  // 列出内置插件并加载可执行文件旁的插件库
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();  // 读取当前已注册的 MuJoCo 插件数量
  if (nplugin) {  // 存在内置插件时打印插件列表
    std::printf("Built-in plugins:\n");  // 输出内置插件列表标题
    for (int i = 0; i < nplugin; ++i) { std::printf("    %s\n", mjp_getPluginAtSlot(i)->name); }  // 遍历注册槽位并打印每个插件的名称
  }  // 结束当前函数、命名空间或控制分支的作用域

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)  // 仅在 Windows 或 Cygwin 平台编译下面的分支
  const std::string sep = "\\";  // 为 Windows 插件路径使用反斜杠分隔符
#else  // 切换到当前预处理条件不成立时使用的平台实现
  const std::string sep = "/";  // 为 POSIX 插件路径使用正斜杠分隔符
#endif  // 结束当前平台条件编译分支


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();  // 取得当前可执行文件所在目录
  if (executable_dir.empty()) { return; }  // 不能确定目录时停止自动加载插件

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;  // 拼接可执行文件目录和插件子目录名
  mj_loadAllPluginLibraries(  // 开始调用 MuJoCo 的批量插件库加载接口
      plugin_dir.c_str(),  // 将插件目录转换为 C 风格字符串作为加载路径
      +[](const char* filename, int first, int count) {  // 提供无捕获加载回调，接收库名及新增插件槽位范围
        std::printf("Plugins registered by library '%s':\n", filename);  // 打印刚加载的插件库文件名
        for (int i = first; i < first + count; ++i) {  // 遍历本次插件库新注册的全部槽位
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);  // 打印当前新增插件的名称
        }  // 结束当前函数、命名空间或控制分支的作用域
      });  // 结束回调定义或参数列表，并完成外层函数调用
}  // 结束当前函数、命名空间或控制分支的作用域


//------------------------------------------- simulation -------------------------------------------

// 禁用自动重置时，检查数值发散警告；无异常返回 nullptr，否则返回警告文本。
const char* Diverged(int disableflags, const mjData* d) {  // 检查数值发散警告，返回警告文本或空指针
  if (disableflags & mjDSBL_AUTORESET) {  // 仅在关闭自动重置时检查是否存在数值发散警告
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {  // 依次检查异常加速度、速度和位置警告
      if (d->warning[w].number > 0) { return mju_warningText(w, d->warning[w].lastinfo); }  // 发现警告记录时返回相应的可读错误说明
    }  // 结束当前函数、命名空间或控制分支的作用域
  }  // 结束当前函数、命名空间或控制分支的作用域
  return nullptr;  // 返回空指针，表示无结果、无警告或本次加载失败，取决于所在函数
}  // 结束当前函数、命名空间或控制分支的作用域

// 按扩展名加载/编译模型，将错误或警告写入查看器。
// 成功返回新模型（由调用方释放）；失败返回 nullptr。
mjModel* LoadModel(const char* file, mj::Simulate& sim) {  // 按文件类型加载模型并将错误或警告传给查看器
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];  // 创建符合官方查看器路径长度上限的字符数组
  mju::strcpy_arr(filename, file);  // 将输入模型路径安全复制到固定容量数组

  // make sure filename is not empty
  if (!filename[0]) { return nullptr; }  // 模型路径为空时直接报告未加载模型

  // load and compile
  char loadError[kErrorLength] = "";  // 创建并清空模型加载错误缓冲区

  mjModel* mnew       = 0;  // 初始化待加载的新模型指针
  auto     load_start = mj::Simulate::Clock::now();  // 记录模型加载开始时的单调时钟时间

  std::string filename_str(filename);  // 将模型路径转换为便于查找扩展名的字符串
  std::string extension;  // 准备保存包含句点的文件扩展名
  size_t      dot_pos = filename_str.rfind('.');  // 从后向前查找文件路径中最后一个句点的位置

  if (dot_pos != std::string::npos && dot_pos < filename_str.length() - 1) {  // 仅当句点存在且后面有字符时提取扩展名
    extension = filename_str.substr(dot_pos);  // 截取最后一个句点开始的扩展名
  }  // 结束当前函数、命名空间或控制分支的作用域

  if (extension == ".mjb") {  // 对 MuJoCo 二进制模型采用直接加载流程
    mnew = mj_loadModel(filename, nullptr);  // 从磁盘加载已编译的 MJB 模型
    if (!mnew) { mju::strcpy_arr(loadError, "could not load binary model"); }  // 二进制加载失败时填写错误说明
  } else if (extension == ".xml") {  // 对 XML 模型采用解析并编译流程
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);  // 加载并编译 XML，同时接收错误或警告信息
  } else {  // 当前条件不满足时，进入另一处理分支
    mjVFS vfs;  // 声明 MuJoCo 虚拟文件系统对象供通用解析器使用
    mj_defaultVFS(&vfs);  // 初始化虚拟文件系统对象
    mjSpec* spec = mj_parse(filename, nullptr, &vfs, loadError, kErrorLength);  // 将其他受支持格式解析为可编译的模型规格
    if (!spec) {  // 解析器未生成模型规格时处理失败信息
      if (!loadError[0]) { mju::strcpy_arr(loadError, "could not parse model"); }  // 若解析器未提供详细错误，则补充通用错误信息
    } else {  // 当前条件不满足时，进入另一处理分支
      mnew = mj_compile(spec, &vfs);  // 将解析得到的模型规格编译为可仿真的 mjModel
      if (!mnew) { mju::strcpy_arr(loadError, mjs_getError(spec)); }  // 模型编译失败时复制规格对象中的错误说明
      mj_deleteSpec(spec);  // 释放临时模型规格对象
    }  // 结束当前函数、命名空间或控制分支的作用域
    mj_deleteVFS(&vfs);  // 释放通用解析流程使用的虚拟文件系统资源
  }  // 结束当前函数、命名空间或控制分支的作用域

  // remove trailing newline character from loadError
  if (loadError[0]) {  // 错误缓冲区非空时处理其中的信息
    int error_length = mju::strlen_arr(loadError);  // 计算错误字符串的实际长度
    if (loadError[error_length - 1] == '\n') { loadError[error_length - 1] = '\0'; }  // 去掉错误信息末尾的换行符，便于界面排版
  }  // 结束当前函数、命名空间或控制分支的作用域

  auto   load_interval = mj::Simulate::Clock::now() - load_start;  // 计算模型加载所用的实际时间间隔
  double load_seconds  = Seconds(load_interval).count();  // 将加载耗时转换为双精度秒数

  if (!mnew) {  // 未获得有效新模型时进入加载失败分支
    std::printf("%s\n", loadError);  // 在终端打印本次模型加载错误
    mju::strcpy_arr(sim.load_error, loadError);  // 将加载错误、警告或耗时提示传给官方查看器
    return nullptr;  // 返回空指针，表示无结果、无警告或本次加载失败，取决于所在函数
  }  // 结束当前函数、命名空间或控制分支的作用域

  // compiler warning: print and pause
  if (loadError[0]) {  // 错误缓冲区非空时处理其中的信息
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);  // 模型已编译但存在警告时打印说明
    sim.run = 0;  // 将仿真设为暂停，等待用户处理警告或异常
  }  // 结束当前函数、命名空间或控制分支的作用域

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25) {  // 加载成功且耗时超过四分之一秒时报告耗时
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);  // 将加载耗时格式化为界面提示文字
  }  // 结束当前函数、命名空间或控制分支的作用域

  mju::strcpy_arr(sim.load_error, loadError);  // 将加载错误、警告或耗时提示传给官方查看器

  return mnew;  // 返回新模型指针，后续由物理线程管理其生命周期
}  // 结束当前函数、命名空间或控制分支的作用域

// simulate in background thread (while rendering in main thread)
// 后台物理循环：处理模型重载、暂停、实时同步和遥测，直到界面请求退出。
void PhysicsLoop(mj::Simulate& sim) {  // 运行后台物理循环，处理加载、暂停、实时同步及采样
  // cpu-sim synchronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;  // 保存实际时钟与仿真时钟的同步基准点

  mjtNum syncSim = 0;  // 保存同步基准对应的仿真时间

  int last_run = -1;  // 保存上次运行状态，-1 表示尚未进入循环
  // 每个运行循环维护输出时间；负值保证首次物理步立即输出。
  mjtNum last_print_time = -1;  // 保存上次遥测输出时间，负值保证首次立即输出

  // run until asked to exit
  while (!sim.exitrequest.load()) {  // 在界面尚未请求退出时持续运行物理线程
    // 处理拖放加载：成功创建新状态后替换旧模型，并恢复 10° 初始倾角。
    if (sim.droploadrequest.load()) {  // 检查是否收到拖放模型文件的原子请求
      sim.LoadMessage(sim.dropfilename);  // 通知界面显示拖放模型的加载提示
      mjModel* mnew = LoadModel(sim.dropfilename, sim);  // 尝试加载拖放文件并获得新模型
      sim.droploadrequest.store(false);  // 清除已消费的拖放加载请求

      mjData* dnew = nullptr;  // 准备保存新模型对应的动态状态对象
      if (mnew) dnew = mj_makeData(mnew);  // 新模型有效时为其分配仿真状态
      if (dnew) {  // 只有新状态创建成功时才替换当前模型
        sim.Load(mnew, dnew, sim.dropfilename);  // 请求渲染线程接收拖放模型及其状态

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);  // 持有查看器递归锁，保证模型状态与界面同步访问

        mj_deleteData(d);  // 释放不再使用的动态状态对象
        mj_deleteModel(m);  // 释放不再使用的模型对象

        m = mnew;  // 将当前模型指针切换到新加载的模型
        d = dnew;  // 将当前动态状态指针切换到新模型的数据对象
        pendulum::SetInitialPose(m, d);  // 清除积分并恢复测试指定的初始倾角

      } else {  // 当前条件不满足时，进入另一处理分支
        sim.LoadMessageClear();  // 清除未成功完成的加载提示
      }  // 结束当前函数、命名空间或控制分支的作用域
    }  // 结束当前函数、命名空间或控制分支的作用域

    // 处理界面 Reload 请求，与拖放加载采用相同的初始化流程。
    if (sim.uiloadrequest.load()) {  // 检查官方界面是否请求重新加载当前模型
      sim.uiloadrequest.fetch_sub(1);  // 原子递减待处理的界面加载请求计数
      sim.LoadMessage(sim.filename);  // 显示当前模型的重新加载提示
      mjModel* mnew = LoadModel(sim.filename, sim);  // 从当前模型文件路径重新读取并编译模型
      mjData*  dnew = nullptr;  // 准备保存重新加载模型对应的动态状态对象
      if (mnew) dnew = mj_makeData(mnew);  // 新模型有效时为其分配仿真状态
      if (dnew) {  // 只有新状态创建成功时才替换当前模型
        sim.Load(mnew, dnew, sim.filename);  // 请求界面接收重新加载的模型与状态

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);  // 持有查看器递归锁，保证模型状态与界面同步访问

        mj_deleteData(d);  // 释放不再使用的动态状态对象
        mj_deleteModel(m);  // 释放不再使用的模型对象

        m = mnew;  // 将当前模型指针切换到新加载的模型
        d = dnew;  // 将当前动态状态指针切换到新模型的数据对象
        pendulum::SetInitialPose(m, d);  // 清除积分并恢复测试指定的初始倾角

      } else {  // 当前条件不满足时，进入另一处理分支
        sim.LoadMessageClear();  // 清除未成功完成的加载提示
      }  // 结束当前函数、命名空间或控制分支的作用域
    }  // 结束当前函数、命名空间或控制分支的作用域

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {  // 运行且启用忙等待时，选择更积极的调度方式
      std::this_thread::yield();  // 主动让出一次处理器时间片，而不固定休眠
    } else {  // 当前条件不满足时，进入另一处理分支
      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 休眠一毫秒，为界面线程让出处理器资源
    }  // 结束当前函数、命名空间或控制分支的作用域

    {  // 建立局部作用域，限制临时变量或锁的生命周期
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);  // 持有查看器递归锁，保证模型状态与界面同步访问

      // run only if model is present
      if (m) {  // 仅在模型已加载时处理运行或暂停状态
        // reset timers on transition between running and paused
        if (sim.run != last_run) {  // 检查仿真是否在运行与暂停之间发生切换
          if (last_run != -1) {  // 首次进入循环后，状态切换才需要清空旧计时数据
            std::memset(d->timer, 0, sizeof(d->timer));  // 清空 MuJoCo 本轮性能计时统计
            std::memset(sim.timer_prev_, 0, sizeof(sim.timer_prev_));  // 清空查看器保存的上一轮计时统计
          }  // 结束当前函数、命名空间或控制分支的作用域
          last_run = sim.run;  // 记住本轮运行状态供下轮比较
        }  // 结束当前函数、命名空间或控制分支的作用域

        // running
        if (sim.run) {  // 仿真处于运行状态时才推进物理时间
          bool stepped = false;  // 记录本轮是否完成了有效的物理推进

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();  // 记录本轮计算开始的实际时钟时间

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;  // 计算距上次同步基准的实际时间
          double     elapsedSim = d->time - syncSim;  // 计算距上次同步基准的仿真时间

          // requested slow-down factor
          // 界面速度百分比换算为墙钟时间/仿真时间，例如 50% 对应 2。
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];  // 由界面速度百分比计算实际时间与仿真时间的比例

          // misalignment condition: distance from target sim time is bigger than syncMisalign
          bool misaligned =  // 准备判断两类时间是否已偏离允许的同步范围
              std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;  // 将实际时间按速度缩放，并与仿真时间比较偏差阈值

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 ||  // 仿真时间回退时，需要重新建立同步基准
              elapsedCPU.count() < 0 ||  // 实际时间出现回退时，也需要重新同步
              syncCPU.time_since_epoch().count() == 0 ||  // 同步时钟尚未初始化时，也需要建立基准
              misaligned ||  // 时间偏差过大时也重新同步
              sim.speed_changed) {  // 用户改变仿真速度时进入重新同步分支
            // re-sync
            syncCPU           = startCPU;  // 将当前实际时间作为新的同步基准
            syncSim           = d->time;  // 将当前仿真时间与新的实际时间基准配对
            sim.speed_changed = false;  // 清除已处理的速度变化标记

            // inject noise
            sim.InjectNoise(sim.key);  // 运行官方控制噪声逻辑；本项目电机控制随后由 PID 回调覆盖

            // 重置、变速或时间偏差过大时重新同步，先推进一步。
            // mj_step 内部调用已注册的 PD 回调，无需在此重复计算力矩。
            pendulum::BeforePhysicsStep(m, d);  // 在推进前检测重置并且仅更新一次 PID 积分
            mj_step(m, d);  // 按模型步长推进物理仿真，内部调用已注册的控制回调
            pendulum::RecordResponse(m, d);  // 按采样间隔保存当前角度、目标角度和控制力矩
            mj_kinematics(m, d);  // 根据推进后的位置更新刚体运动学结果
            pendulum::PrintTelemetry(m, d, last_print_time);  // 按输出间隔向终端打印最新状态
            const char* message = Diverged(m->opt.disableflags, d);  // 检查当前模型选项和状态中是否有数值发散警告
            if (message) {  // 存在发散信息时进入暂停和提示流程
              sim.run = 0;  // 将仿真设为暂停，等待用户处理警告或异常
              mju::strcpy_arr(sim.load_error, message);  // 将发散警告文本复制到界面的错误提示区
            } else {  // 当前条件不满足时，进入另一处理分支
              stepped = true;  // 标记本轮已完成可保存到历史记录的物理步
            }  // 结束当前函数、命名空间或控制分支的作用域
          }  // 结束当前函数、命名空间或控制分支的作用域

          // in-sync: step until ahead of cpu
          else {  // 前面的条件不满足时进入备用处理分支
            bool   measured = false;  // 记录本轮是否已更新实际减速比例统计
            mjtNum prevSim  = d->time;  // 保存本轮开始的仿真时间，用于检测中途重置

            double refreshTime = simRefreshFraction / sim.refresh_rate;  // 计算本轮允许使用的物理计算时间预算

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&  // 当缩放后的仿真进度落后于实际时钟时继续追赶
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {  // 同时确保本轮计算尚未超过界面刷新时间预算
              // measure slowdown before first step
              if (!measured && elapsedSim) {  // 存在有效仿真时间差且尚未统计时，测量实际减速比例
                sim.measured_slowdown =  // 准备更新官方界面显示的实际减速比例
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;  // 用实际经过秒数除以仿真经过秒数得到减速比例
                measured = true;  // 避免在同一轮追赶循环中重复测量减速比例
              }  // 结束当前函数、命名空间或控制分支的作用域

              // inject noise
              sim.InjectNoise(sim.key);  // 运行官方控制噪声逻辑；本项目电机控制随后由 PID 回调覆盖

              // 追赶实时进度；每步内部调用 PD 回调，随后更新位姿并输出遥测。
              pendulum::BeforePhysicsStep(m, d);  // 在推进前检测重置并且仅更新一次 PID 积分
              mj_step(m, d);  // 按模型步长推进物理仿真，内部调用已注册的控制回调
              pendulum::RecordResponse(m, d);  // 按采样间隔保存当前角度、目标角度和控制力矩
              mj_kinematics(m, d);  // 根据推进后的位置更新刚体运动学结果
              pendulum::PrintTelemetry(m, d, last_print_time);  // 按输出间隔向终端打印最新状态
              const char* message = Diverged(m->opt.disableflags, d);  // 检查当前模型选项和状态中是否有数值发散警告
              if (message) {  // 存在发散信息时进入暂停和提示流程
                sim.run = 0;  // 将仿真设为暂停，等待用户处理警告或异常
                mju::strcpy_arr(sim.load_error, message);  // 将发散警告文本复制到界面的错误提示区
              } else {  // 当前条件不满足时，进入另一处理分支
                stepped = true;  // 标记本轮已完成可保存到历史记录的物理步
              }  // 结束当前函数、命名空间或控制分支的作用域

              // break if reset
              if (d->time < prevSim) { break; }  // 检测到仿真时间重置时退出当前追赶循环
            }  // 结束当前函数、命名空间或控制分支的作用域
          }  // 结束当前函数、命名空间或控制分支的作用域

          // save current state to history buffer
          if (stepped) { sim.AddToHistory(); }  // 本轮确实推进成功后，将当前状态加入官方历史缓冲
        }  // 结束当前函数、命名空间或控制分支的作用域

        // paused
        else {  // 前面的条件不满足时进入备用处理分支
          // 暂停时仅前向计算以更新画面和关节滑块，不推进仿真时间。
          // mj_forward 仍可触发控制回调，但此处不打印遥测。
          mj_forward(m, d);  // 更新派生运动学和动力学状态，不推进仿真时间
          if (sim.pause_update) { mju_copy(d->qacc_warmstart, d->qacc, m->nv); }  // 暂停更新开启时，将加速度复制为求解器热启动状态
          sim.speed_changed = true;  // 要求下次继续运行时重新对齐仿真时间和实际时间
        }  // 结束当前函数、命名空间或控制分支的作用域
      }  // 结束当前函数、命名空间或控制分支的作用域
    }  // release std::lock_guard<std::mutex>  // 结束当前函数、命名空间或控制分支的作用域
  }  // 结束当前函数、命名空间或控制分支的作用域
}  // 结束当前函数、命名空间或控制分支的作用域
}  // namespace  // 结束当前函数、命名空间或控制分支的作用域

//-------------------------------------- physics_thread --------------------------------------------

// 物理线程入口：加载初始模型、运行循环，退出时释放模型和状态。
void PhysicsThread(mj::Simulate* sim, const char* filename) {  // 后台线程入口，管理初次模型加载、物理循环及资源清理
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {  // 只有提供模型路径时才执行初次自动加载
    sim->LoadMessage(filename);  // 通知主线程显示初次模型加载提示
    m = LoadModel(filename, *sim);  // 加载初始模型，并将错误信息传给界面
    if (m) {  // 仅在模型已加载时处理运行或暂停状态
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);  // 持有查看器递归锁，使模型状态修改与渲染同步

      d = mj_makeData(m);  // 为当前模型分配动态仿真状态
    }  // 结束当前函数、命名空间或控制分支的作用域
    if (d) {  // 动态状态创建成功时将模型交给查看器显示
      sim->Load(m, d, filename);  // 请求主线程加载当前模型和状态并显示文件名

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);  // 持有查看器递归锁，使模型状态修改与渲染同步

      pendulum::SetInitialPose(m, d);  // 清除积分并恢复测试指定的初始倾角

    } else {  // 当前条件不满足时，进入另一处理分支
      sim->LoadMessageClear();  // 初次加载失败时清除加载提示
    }  // 结束当前函数、命名空间或控制分支的作用域
  }  // 结束当前函数、命名空间或控制分支的作用域

  PhysicsLoop(*sim);  // 进入持续运行的后台物理循环，直到收到退出请求

  // delete everything we allocated
  mj_deleteData(d);  // 释放不再使用的动态状态对象
  mj_deleteModel(m);  // 释放不再使用的模型对象
}  // 结束当前函数、命名空间或控制分支的作用域

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)  // 仅为启用 AVX 的 macOS 构建编译 Rosetta 兼容性检查
extern void        DisplayErrorDialogBox(const char* title, const char* msg);  // 声明 macOS 兼容性错误弹窗接口
static const char* rosetta_error_msg = nullptr;  // 保存 Rosetta 兼容性错误文本，初始没有错误
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {  // 导出供 MuJoCo 调用的 Rosetta 错误回调并阻止链接器移除
  rosetta_error_msg = msg;  // 记录 MuJoCo 提供的 Rosetta 错误字符串
}  // 结束当前函数、命名空间或控制分支的作用域
#endif  // 结束当前平台条件编译分支

// run event loop
// 应用运行入口：检查库版本、注册控制回调、创建界面并管理物理线程。
// filename 由 main.cpp 选择；渲染循环阻塞至窗口关闭。
int RunSimulation(const char* filename) {  // 创建官方查看器并管理物理线程，直到用户关闭窗口

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)  // 仅为启用 AVX 的 macOS 构建编译 Rosetta 兼容性检查
  if (rosetta_error_msg) {  // 存在 Rosetta 兼容性错误时显示提示并退出
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);  // 弹窗解释当前 Rosetta 2 运行方式不受支持
    std::exit(1);  // 以失败状态码立即终止进程
  }  // 结束当前函数、命名空间或控制分支的作用域
#endif  // 结束当前平台条件编译分支

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());  // 打印当前实际加载的 MuJoCo 动态库版本
  if (mjVERSION_HEADER != mj_version()) {  // 检查编译使用的头文件版本是否与运行时库一致
    mju_error("Headers and library have different versions");  // 发现版本不一致时报告不可恢复错误
  }  // 结束当前函数、命名空间或控制分支的作用域

  // MuJoCo 全局控制回调，在物理推进和前向计算时计算控制力矩。
  mjcb_control = pendulum::PendulumControl;  // 将项目 PID 函数注册为 MuJoCo 全局控制回调

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();  // 扫描并加载本程序旁的 MuJoCo 插件

  mjvCamera cam;  // 声明控制三维视角的相机结构
  mjv_defaultCamera(&cam);  // 初始化相机结构为 MuJoCo 默认值

  mjvOption opt;  // 声明三维场景可视化选项结构
  mjv_defaultOption(&opt);  // 初始化场景可视化选项

  mjvPerturb pert;  // 声明鼠标交互扰动状态结构
  mjv_defaultPerturb(&pert);  // 初始化鼠标交互扰动状态

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(  // 创建由智能指针管理的官方查看器对象
          std::make_unique<mj::GlfwAdapter>(),  // 向查看器传入 GLFW 平台窗口与交互适配器
      &cam,  // 向查看器传入相机配置地址
      &opt,  // 向查看器传入场景可视化选项地址
      &pert,  // 向查看器传入交互扰动状态地址
      /* is_passive = */ false);  // 采用主动仿真模式，由本程序而非外部调用方推进物理状态

  pendulum::InstallDashboard(*sim);  // 加载 PID 配置并安装参数面板和实时曲线扩展

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);  // 启动物理后台线程，传入查看器指针和模型路径

  // start simulation UI loop (blocking call)
  sim->RenderLoop();  // 在主线程运行界面事件及渲染循环，直到窗口关闭
  // 界面关闭会发出退出请求；等待物理线程完成清理后再销毁界面。
  physicsthreadhandle.join();  // 等待物理线程退出并完成模型资源清理

  // 线程已退出，解除全局控制回调。
  mjcb_control = nullptr;  // 解除全局控制回调，避免仿真结束后继续引用该控制器
  return 0;  // 以成功状态结束当前程序或运行入口
}  // 结束当前函数、命名空间或控制分支的作用域
