#include "telemetry.h"  // 引入终端遥测输出接口声明

#include <cstdio>  // 引入格式化终端输出及缓冲刷新函数

namespace pendulum {  // 进入项目的倒立摆功能命名空间
// 由物理线程在推进后调用；暂停分支不调用，因此暂停时不持续输出。
void PrintTelemetry(const mjModel* m, const mjData* d, mjtNum& last_time) {  // 按仿真时间间隔输出当前状态并更新上次输出时间
  const int joint = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");  // 按 XML 名称查找倒立摆关节编号
  const int motor = mj_name2id(m, mjOBJ_ACTUATOR, "pendulum_motor");  // 按 XML 名称查找倒立摆电机执行器编号
  // 非倒立摆模型不输出，避免读取无效状态。
  if (joint < 0 || motor < 0) return;  // 模型缺少指定对象时跳过本次处理，避免无效下标
  // 限制输出频率；首次调用或重置后时间回退时，不受该间隔限制。
  if (last_time >= 0 && d->time >= last_time && d->time - last_time < 0.02) return;  // 距上次输出不足 20ms 且时间未回退时跳过输出
  last_time = d->time;  // 记录本次输出的仿真时间，供下次限频和重置检测使用
  // 角度转为 °；角速度保持 rad/s；ctrl 为控制指令（当前模型单位为 Nm）。
  std::printf("时间: %.3fs | 角度: %.3f° | 角速度: %.3frad/s | 力矩: %.3fNm\n",  // 按固定格式输出时间、角度、角速度和控制力矩
              static_cast<double>(d->time),  // 提供当前仿真时间，单位为秒
              static_cast<double>(d->qpos[m->jnt_qposadr[joint]] * 180 / mjPI),  // 按关节位置地址读取角度，并从 rad 转换为 °
              static_cast<double>(d->qvel[m->jnt_dofadr[joint]]),  // 按关节速度地址读取角速度，单位为 rad/s
              static_cast<double>(d->ctrl[motor]));  // 提供当前执行器控制指令，当前模型中单位为 Nm
  // 立即刷新缓冲，使系统终端中的数据实时可见。
  std::fflush(stdout);  // 立即刷新标准输出缓冲，使终端实时显示信息
}  // 结束当前函数、命名空间或控制分支的作用域

}  // namespace pendulum  // 结束当前函数、命名空间或控制分支的作用域
