#include "pendulum_control.h"  // 引入 PID 参数与控制器接口声明
#include <cmath>  // 引入绝对值和有限数检查等数学函数
#include <cstdio>  // 引入格式化终端输出及缓冲刷新函数
#include <stdexcept>  // 引入测试失败时使用的异常类型
#include <filesystem>  // 引入临时文件路径及文件删除工具
#include <chrono>  // 引入单调时钟和时间间隔类型
#include <fstream>  // 引入文本文件输入输出流

namespace {  // 进入匿名命名空间，使内部工具及状态仅在本文件可见
void Check(bool ok, const char* message) {  // 测试断言：条件不满足时抛出带错误说明的异常
  if (!ok) throw std::runtime_error(message);  // 断言条件失败时抛出异常，使测试以失败结束并显示原因
}  // 结束当前函数、命名空间或控制分支的作用域
}  // 结束当前函数、命名空间或控制分支的作用域
int main() {  // 程序入口，负责选择模型并启动仿真；测试文件中用于执行回归验证
  char error[1024] = {};  // 创建清零的模型加载错误信息缓冲区
  mjModel* m = mj_loadXML(TEST_MODEL_PATH, nullptr, error, sizeof(error));  // 加载测试模型，路径由 CMake 以编译宏提供
  Check(m != nullptr, error);  // 确认测试模型加载成功，否则报告 MuJoCo 的错误信息
  mjData* d = mj_makeData(m);  // 为测试模型创建动态状态对象
  mjcb_control = pendulum::PendulumControl;  // 将项目 PID 函数注册为 MuJoCo 全局控制回调
  pendulum::PidParameters p;  // 创建使用默认增益和初始条件的测试参数
  pendulum::SetParameters(p);  // 将当前测试参数应用到控制器
  pendulum::SetInitialPose(m, d);  // 清除积分并恢复测试指定的初始倾角
  Check(std::abs(d->qpos[1] - 10*mjPI/180) < 1e-12, "Initial inclination");  // 验证：初始倾角应精确等于 10°
  for (int i = 0; i < 5000; ++i) {  // 按 2ms 步长执行 5000 步，对应十秒默认 PD 响应
    pendulum::AdvanceIntegral(m, d);  // 在物理步前更新一次积分状态
    mj_step(m, d);  // 按模型步长推进物理仿真，内部调用已注册的控制回调
  }  // 结束当前函数、命名空间或控制分支的作用域
  Check(std::abs(d->qpos[1]) < 0.005 && std::abs(d->qpos[0]) < 0.03, "Default PD convergence");  // 验证：默认 PD 运行十秒后角度应接近零

  std::printf("10s: x=%g theta=%g\n", d->qpos[0], d->qpos[1]);
  mj_resetData(m, d);
  // 前向计算/多次回调不得重复积分；只有显式物理步前才累积。
  p.ki = 2;  // 开启积分反馈，将积分增益设为 2
  pendulum::SetParameters(p);  // 将当前测试参数应用到控制器
  d->qpos[1] = 0.01;  // 为单关节测试设置 0.01rad 的小角度误差
  d->qvel[1] = 0;  // 清除角速度，隔离比例项和积分项的影响
  pendulum::AdvanceIntegral(m, d);  // 在物理步前更新一次积分状态
  const double value = pendulum::GetIntegral();  // 保存一次积分更新后的数值，供后续不变性检查
  Check(std::abs(value - 0.01*m->opt.timestep) < 1e-12, "Single-step integral");  // 验证：积分增量应等于负角度误差乘以模型步长
  for (int i = 0; i < 10; ++i) mj_forward(m, d);  // 连续执行十次不推进时间的前向计算，模拟暂停时的回调
  Check(pendulum::GetIntegral() == value, "Paused callback changed integral");  // 验证：多次前向计算不应改变已保存的积分状态
  pendulum::ResetIntegral();  // 显式清零积分状态，为后续测试建立可比初始条件
  Check(pendulum::GetIntegral() == 0, "Integral reset");  // 验证：积分清零接口应使积分值恢复为零

  // 大误差下输出饱和，误差积分不得继续向饱和方向累积。
  p.kp = 200; p.ki = 100; p.kd = 0; p.force_limit = 1;  // 设置高比例、高积分、无微分及 1Nm 限幅，构造输出饱和工况
  pendulum::SetParameters(p);  // 将当前测试参数应用到控制器
  d->qpos[1] = 1;  // 将测试角度设为 1rad，制造较大的负向误差
  for (int i = 0; i < 100; ++i) pendulum::AdvanceIntegral(m, d);  // 在饱和工况下重复尝试积分，验证抗饱和逻辑
  pendulum::PendulumControl(m, d);  // 直接调用控制回调，计算当前工况下的限幅力矩
  Check(d->ctrl[0] == 1, "Torque limit");  // 验证：输出控制力矩应被用户设置限制为 -1Nm
  Check(pendulum::GetIntegral() == 0, "Anti-windup");  // 验证：饱和且误差加重饱和时，积分应保持为零
  p.force_limit = 20;  // 放宽用户力矩限制，以单独测试 XML 控制范围约束
  pendulum::SetParameters(p);  // 将当前测试参数应用到控制器
  m->actuator_ctrlrange[1] = 0.5;  // 临时将模型执行器的负向控制下界收紧到 -0.5Nm
  pendulum::PendulumControl(m, d);  // 直接调用控制回调，计算当前工况下的限幅力矩
  Check(d->ctrl[0] == 0.5, "XML actuator limit");  // 验证：即使用户限幅较宽，输出仍应遵守 XML 的 -0.5Nm 下界
  m->actuator_ctrlrange[1] = 20;  // 恢复原模型的负向控制下界，避免影响后续动力学测试

  // 开启积分后的完整动力学响应必须保持有限并回到目标附近。
  p = {}; p.ki = 2;  // 先恢复默认参数，再开启增益为 2 的积分项
  pendulum::SetParameters(p);  // 将当前测试参数应用到控制器
  mj_resetData(m, d);  // 将时间、速度及其他测试状态重置为模型默认值
  pendulum::SetInitialPose(m, d);  // 清除积分并恢复测试指定的初始倾角
  for (int i = 0; i < 15000; ++i) {  // 按 2ms 步长执行 15000 步，观察三十秒完整 PID 响应
    pendulum::AdvanceIntegral(m, d);  // 在物理步前更新一次积分状态
    mj_step(m, d);  // 按模型步长推进物理仿真，内部调用已注册的控制回调
    Check(std::isfinite(d->qpos[1]) && std::abs(d->ctrl[0]) <= 20, "PID bounded response");  // 验证：完整 PID 运行中角度必须为有限数且输出不能超过 20Nm
  }  // 结束当前函数、命名空间或控制分支的作用域
  Check(std::abs(d->qpos[1]) < 0.01, "PID convergence");  // 验证：开启积分后运行三十秒，角度应回到目标附近

  // 双向倾角及小车偏移：检查整个运动过程，而不只检查最终角度。
  Check(m->nq == 2 && m->nv == 2 && m->nu == 1, "Two DOFs and one actuator");
  for (double angle : {-10.0, 10.0}) {
    p = {}; p.initial_deg = angle;
    pendulum::SetParameters(p);
    mj_resetData(m, d);
    pendulum::SetInitialPose(m, d);
    d->qpos[0] = 0.4;
    d->qvel[0] = -0.2;
    for (int i = 0; i < 15000; ++i) {
      pendulum::AdvanceIntegral(m, d);
      mj_step(m, d);
      Check(std::isfinite(d->qpos[1]) && std::abs(d->qpos[1]) < 0.5,
            "Pendulum fell during recovery");
      Check(std::abs(d->qpos[0]) < 2.3 && std::abs(d->ctrl[0]) <= 20,
            "Cart exceeded travel or force limit");
    }
    Check(std::abs(d->qpos[0]) < 0.01 && std::abs(d->qpos[1]) < 0.001 &&
          std::abs(d->qvel[0]) < 0.01, "Cart and pendulum convergence");
  }

  // 参数保存/恢复，以及损坏文件不覆盖当前参数。
  const auto path = std::filesystem::temp_directory_path() /  // 在系统临时目录下准备本次参数读写测试的文件路径
      ("pendulum_pid_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".txt");  // 用单调时钟计数生成文件名，降低并发测试时的命名冲突
  Check(pendulum::SaveParameters(path.c_str()), "Save parameters");  // 验证：参数文件应能成功写入并关闭
  auto changed = p; changed.kp = 50;  // 复制已保存的参数并修改比例增益，用于验证加载确实恢复旧值
  pendulum::SetParameters(changed);  // 将修改后的参数提交给控制器，制造保存值与当前值的差异
  Check(pendulum::LoadParameters(path.c_str()), "Load parameters");  // 验证：有效参数文件应能成功加载
  Check(pendulum::GetParameters().kp == p.kp && pendulum::GetParameters().ki == p.ki, "Persistence round trip");  // 验证：保存再加载后，比例和积分增益应恢复原值
  { std::ofstream out(path); out << "invalid parameters\n"; }  // 将参数文件改为无效文本，测试损坏文件的拒绝加载行为
  Check(!pendulum::LoadParameters(path.c_str()), "Reject corrupt parameters");  // 验证：损坏的参数文件必须被拒绝
  Check(pendulum::GetParameters().kp == p.kp, "Corrupt file changed parameters");  // 验证：拒绝损坏文件后，已有参数不应发生变化
  std::filesystem::remove(path);  // 删除本次测试创建的临时参数文件
  mjcb_control = nullptr;  // 解除全局控制回调，避免仿真结束后继续引用该控制器
  mj_deleteData(d);  // 释放不再使用的动态状态对象
  mj_deleteModel(m);  // 释放不再使用的模型对象
  std::puts("PID tests passed");  // 输出控制器回归测试全部通过的提示
}  // 结束当前函数、命名空间或控制分支的作用域
