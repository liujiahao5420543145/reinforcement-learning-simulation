#pragma once  // 确保同一个头文件在每个编译单元中只展开一次

#include <mujoco/mujoco.h>  // 引入 MuJoCo 模型、状态、动力学及可视化 API

namespace pendulum {  // 进入项目的倒立摆功能命名空间
/**
 * 按仿真时间约每 20ms 输出角度、角速度和控制力矩。
 * model/data 为只读模型和状态；last_time 为调用方保存的上次输出时间，
 * 初值应为 -1。输出后更新 last_time，时间回退时允许立即输出。
 */
void PrintTelemetry(const mjModel* model, const mjData* data, mjtNum& last_time);  // 按仿真时间间隔输出当前状态并更新上次输出时间，此处为对外声明
}  // namespace pendulum  // 结束当前函数、命名空间或控制分支的作用域
