#pragma once  // 确保同一个头文件在每个编译单元中只展开一次
#include <mujoco/mujoco.h>  // 引入 MuJoCo 模型、状态、动力学及可视化 API
namespace mujoco { class Simulate; }  // 前置声明官方查看器类，避免头文件依赖完整实现
namespace pendulum {  // 进入项目的倒立摆功能命名空间
// 安装官方查看器扩展，在主线程创建滑块、按钮和响应图。
void InstallDashboard(mujoco::Simulate& simulation);  // 加载保存参数并注册面板、事件及曲线渲染回调，此处为对外声明
// 在 sim.mtx 保护下由物理线程调用；每个物理步前积分，步后采样。
void BeforePhysicsStep(const mjModel* model, const mjData* data);  // 在物理步前检测实验重置并更新积分，此处为对外声明
void RecordResponse(const mjModel* model, const mjData* data);  // 按采样间隔记录角度、目标及水平驱动力，此处为对外声明
}  // namespace pendulum  // 结束当前函数、命名空间或控制分支的作用域
