#include "pendulum_control.h"  // 引入 PID 参数与控制器接口声明
#include <algorithm>  // 引入最大值、最小值等标准算法
#include <cmath>  // 引入绝对值和有限数检查等数学函数
#include <fstream>  // 引入文本文件输入输出流
#include <iomanip>  // 引入浮点输出精度控制工具
#include <mutex>  // 引入互斥锁和自动加锁工具

namespace pendulum {  // 进入项目的倒立摆功能命名空间
namespace {  // 进入匿名命名空间，使内部工具及状态仅在本文件可见
std::mutex mutex;  // 保护控制参数和积分，防止界面线程与物理线程并发访问
PidParameters parameters;  // 保存当前生效的 PID 参数，初值使用结构体默认值
double integral = 0;  // 保存误差对时间的积分，初值为零，单位为 rad·s
// 同时遵守模型的控制范围；本模型 gear=1，控制值对应 Nm。
bool ReadState(const mjModel* m, const mjData* d, double& error,  // 读取角度误差、角速度和执行器范围，通过引用参数返回结果
               double& velocity, int& motor, double& lower, double& upper) {  // 继续声明输出引用：角速度、执行器编号及控制上下界
  const int joint = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");  // 按 XML 名称查找倒立摆关节编号
  motor = mj_name2id(m, mjOBJ_ACTUATOR, "pendulum_motor");  // 将找到的执行器编号写入输出参数
  if (joint < 0 || motor < 0) return false;  // 模型缺少指定关节或电机时报告读取失败
  error = parameters.target_deg * mjPI / 180 - d->qpos[m->jnt_qposadr[joint]];  // 将目标角度转为 rad，并减去当前关节角度得到误差
  velocity = d->qvel[m->jnt_dofadr[joint]];  // 按关节速度地址读取角速度，单位为 rad/s
  lower = -parameters.torque_limit;  // 设置用户允许的负向控制力矩下界
  upper = parameters.torque_limit;  // 设置用户允许的正向控制力矩上界
  if (m->actuator_ctrllimited[motor]) {  // 若 XML 为该执行器启用了控制限幅，则进一步收紧范围
    lower = std::max(lower, static_cast<double>(m->actuator_ctrlrange[2 * motor]));  // 选择用户下界和模型下界中更严格的一项
    upper = std::min(upper, static_cast<double>(m->actuator_ctrlrange[2 * motor + 1]));  // 选择用户上界和模型上界中更严格的一项
  }  // 结束当前函数、命名空间或控制分支的作用域
  return lower <= upper;  // 只有上下界构成有效区间时才报告成功
}  // 结束当前函数、命名空间或控制分支的作用域
bool Valid(const PidParameters& p) {  // 检查全部 PID 参数是否为有限数且位于允许范围
  return std::isfinite(p.kp) && p.kp >= 0 && p.kp <= 200 &&  // 要求比例增益为有限数，且位于 0 至 200
         std::isfinite(p.ki) && p.ki >= 0 && p.ki <= 100 &&  // 同时要求积分增益为有限数，且位于 0 至 100
         std::isfinite(p.kd) && p.kd >= 0 && p.kd <= 40 &&  // 同时要求微分增益为有限数，且位于 0 至 40
         std::isfinite(p.target_deg) && std::abs(p.target_deg) <= 30 &&  // 同时要求目标角度为有限数，且绝对值不超过 30°
         std::isfinite(p.torque_limit) && p.torque_limit >= 0 && p.torque_limit <= 20 &&  // 同时要求力矩限幅为有限数，且位于 0 至 20 Nm
         std::isfinite(p.initial_deg) && std::abs(p.initial_deg) <= 45;  // 同时要求初始倾角为有限数，且绝对值不超过 45°
}  // 结束当前函数、命名空间或控制分支的作用域
}  // namespace  // 结束当前函数、命名空间或控制分支的作用域

PidParameters GetParameters() {  // 线程安全地读取当前 PID 参数副本
  const std::lock_guard<std::mutex> lock(mutex);  // 在当前作用域自动持有控制器互斥锁，离开作用域时释放
  return parameters;  // 返回当前参数的副本，不暴露共享变量的引用
}  // 结束当前函数、命名空间或控制分支的作用域
void SetParameters(const PidParameters& p) {  // 校验并更新当前 PID 参数
  const std::lock_guard<std::mutex> lock(mutex);  // 在当前作用域自动持有控制器互斥锁，离开作用域时释放
  if (Valid(p)) parameters = p;  // 仅在整组参数有效时覆盖当前配置
}  // 结束当前函数、命名空间或控制分支的作用域
void ResetIntegral() {  // 清零控制器的误差积分
  const std::lock_guard<std::mutex> lock(mutex);  // 在当前作用域自动持有控制器互斥锁，离开作用域时释放
  integral = 0;  // 将误差积分状态清零
}  // 结束当前函数、命名空间或控制分支的作用域
double GetIntegral() {  // 线程安全地读取误差积分
  const std::lock_guard<std::mutex> lock(mutex);  // 在当前作用域自动持有控制器互斥锁，离开作用域时释放
  return integral;  // 返回当前累计的误差积分
}  // 结束当前函数、命名空间或控制分支的作用域
void AdvanceIntegral(const mjModel* m, const mjData* d) {  // 在一个物理步之前执行一次积分及抗饱和判断
  const std::lock_guard<std::mutex> lock(mutex);  // 在当前作用域自动持有控制器互斥锁，离开作用域时释放
  double error, velocity, lower, upper;  // 为误差、角速度和控制上下界准备局部输出变量
  int motor;  // 为执行器编号准备局部输出变量
  if (!ReadState(m, d, error, velocity, motor, lower, upper)) return;  // 读取状态失败或控制范围无效时直接返回
  if (parameters.ki == 0) { integral = 0; return; }  // 关闭积分项时清除旧积分，结束本步积分更新
  const double candidate = integral + error * m->opt.timestep;  // 使用矩形积分法计算本物理步的候选积分值
  const double torque = parameters.kp * error + parameters.ki * candidate - parameters.kd * velocity;  // 计算采用候选积分时的未限幅 PID 力矩
  // 条件积分抗饱和：输出饱和且误差会进一步加剧饱和时停止累积。
  if ((torque > upper && error > 0) || (torque < lower && error < 0)) return;  // 若新积分会进一步加重输出饱和，则放弃本次累积
  integral = candidate;  // 接受本步候选积分值
}  // 结束当前函数、命名空间或控制分支的作用域
void PendulumControl(const mjModel* m, mjData* d) {  // MuJoCo 控制回调，仅计算并写入力矩，不推进积分
  const std::lock_guard<std::mutex> lock(mutex);  // 在当前作用域自动持有控制器互斥锁，离开作用域时释放
  double error, velocity, lower, upper;  // 为误差、角速度和控制上下界准备局部输出变量
  int motor;  // 为执行器编号准备局部输出变量
  if (!ReadState(m, d, error, velocity, motor, lower, upper)) return;  // 读取状态失败或控制范围无效时直接返回
  // 微分项作用于测量速度，调整目标角度时不会产生微分冲击。
  d->ctrl[motor] = mju_clip(parameters.kp * error + parameters.ki * integral - parameters.kd * velocity,  // 将比例项、积分项和负速度反馈相加，交给限幅函数
                           lower, upper);  // 使用当前有效的力矩上下界完成限幅并赋给执行器
}  // 结束当前函数、命名空间或控制分支的作用域
void SetInitialPose(const mjModel* m, mjData* d) {  // 设置实验初始倾角并更新派生状态
  ResetIntegral();  // 清除上一阶段或上一实验的积分状态
  const auto p = GetParameters();  // 读取一份线程安全的当前参数快照
  const int joint = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");  // 按 XML 名称查找倒立摆关节编号
  if (joint >= 0) d->qpos[m->jnt_qposadr[joint]] = p.initial_deg * mjPI / 180;  // 找到关节时，将设置的初始角度由 ° 转为 rad 并写入位置数组
  mj_forward(m, d);  // 更新派生运动学和动力学状态，不推进仿真时间
}  // 结束当前函数、命名空间或控制分支的作用域
bool SaveParameters(const char* path) {  // 将当前控制参数保存到指定文本路径
  const auto p = GetParameters();  // 读取一份线程安全的当前参数快照
  std::ofstream out(path);  // 打开指定路径的输出文件，准备保存参数
  out << std::setprecision(17) << p.kp << ' ' << p.ki << ' ' << p.kd << ' '  // 以 17 位有效数字依次写入比例、积分和微分增益
      << p.target_deg << ' ' << p.torque_limit << ' ' << p.initial_deg << '\n';  // 继续写入目标角度、力矩限幅和初始倾角，并结束该行
  out.close();  // 关闭文件并提交输出缓冲，便于检查写入错误
  return !out.fail();  // 文件打开、写入或关闭未发生失败时返回成功
}  // 结束当前函数、命名空间或控制分支的作用域
bool LoadParameters(const char* path) {  // 读取并校验参数文件，成功后更新参数和清零积分
  PidParameters p;  // 创建具有默认值的参数对象，用于读取或测试
  std::ifstream in(path);  // 打开指定路径的参数输入文件
  if (!(in >> p.kp >> p.ki >> p.kd >> p.target_deg >> p.torque_limit >> p.initial_deg) || !Valid(p)) return false;  // 按固定顺序读取六个参数，读取失败或参数越界时拒绝加载
  in >> std::ws;  // 读取并跳过文件末尾空白字符
  if (!in.eof()) return false;  // 六个参数后还有非空白内容时，判定文件格式无效
  SetParameters(p);  // 将整组经过校验的参数提交给控制器
  ResetIntegral();  // 清除上一阶段或上一实验的积分状态
  return true;  // 报告本次操作成功
}  // 结束当前函数、命名空间或控制分支的作用域
}  // namespace pendulum  // 结束当前函数、命名空间或控制分支的作用域
