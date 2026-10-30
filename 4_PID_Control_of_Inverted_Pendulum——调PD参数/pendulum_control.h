#pragma once  // 确保同一个头文件在每个编译单元中只展开一次
#include <mujoco/mujoco.h>  // 引入 MuJoCo 模型、状态、动力学及可视化 API

namespace pendulum {  // 进入项目的倒立摆功能命名空间
// 参数单位：角度 °、力矩 Nm；内部控制统一使用 rad 和 rad/s。
struct PidParameters {  // 定义一组可保存、可实时更新的实验参数
  double kp = 30;  // 比例增益默认 30，决定角度误差产生的回正力矩
  double ki = 0;  // 积分增益默认 0，即默认关闭积分项
  double kd = 8;  // 微分增益默认 8，通过速度反馈抑制振荡
  double target_deg = 0;  // 目标倾角默认 0°，对应竖直向上
  double torque_limit = 20;  // 用户力矩限幅默认 20 Nm
  double initial_deg = 10;  // 加载或重置实验时的初始倾角默认 10°
};  // 结束结构体或初始化列表定义
PidParameters GetParameters();  // 线程安全地读取当前 PID 参数副本，此处为对外声明
void SetParameters(const PidParameters& parameters);  // 校验并更新当前 PID 参数，此处为对外声明
// 积分仅在显式物理步之前更新，避免暂停前向计算或 RK4 多次回调重复积分。
void AdvanceIntegral(const mjModel* model, const mjData* data);  // 在一个物理步之前执行一次积分及抗饱和判断，此处为对外声明
void ResetIntegral();  // 清零控制器的误差积分，此处为对外声明
double GetIntegral();  // 线程安全地读取误差积分，此处为对外声明
// 纯控制回调：计算 PID 并按用户限制及 XML 执行器限制限幅。
void PendulumControl(const mjModel* model, mjData* data);  // MuJoCo 控制回调，仅计算并写入力矩，不推进积分，此处为对外声明
void SetInitialPose(const mjModel* model, mjData* data);  // 设置实验初始倾角并更新派生状态，此处为对外声明
// 文本参数持久化，失败时返回 false，加载无效内容不修改当前参数。
bool SaveParameters(const char* path);  // 将当前控制参数保存到指定文本路径，此处为对外声明
bool LoadParameters(const char* path);  // 读取并校验参数文件，成功后更新参数和清零积分，此处为对外声明
}  // namespace pendulum  // 结束当前函数、命名空间或控制分支的作用域
