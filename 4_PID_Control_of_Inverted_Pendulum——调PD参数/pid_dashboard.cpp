#include "pid_dashboard.h"  // 引入调参面板与响应采样接口声明
#include "pendulum_control.h"  // 引入 PID 参数与控制器接口声明
#include "simulate.h"  // 引入 MuJoCo 官方查看器类与界面状态
#include "platform_ui_adapter.h"  // 引入查看器平台接口，用于访问当前 OpenGL 绘制上下文

#include <algorithm>  // 引入最大值、最小值等标准算法
#include <cmath>  // 引入绝对值和有限数检查等数学函数
#include <cstdio>  // 引入格式化终端输出及缓冲刷新函数
#include <deque>  // 引入双端队列以保存滚动曲线样本
#include <mutex>  // 引入互斥锁和自动加锁工具

namespace pendulum {  // 进入项目的倒立摆功能命名空间
namespace {  // 进入匿名命名空间，使内部工具及状态仅在本文件可见
// UI 参数仅由渲染线程访问；控制器通过带锁接口接收参数副本。
PidParameters ui_parameters;  // 保存界面线程使用的参数副本，滑块直接绑定其字段
int section = -1;  // 保存 PID 面板在左侧界面中的分区编号，未建立时为 -1
int show_curves = 1;  // 响应曲线显示开关，默认开启
char status[mjMAXUINAME] = "Ready";  // 保存面板状态文本，长度限制与 MuJoCo UI 一致
struct Sample { double time, angle, target, torque; };  // 定义单个采样点：时间、实际角度、目标角度及力矩
std::mutex samples_mutex;  // 保护物理线程写入与渲染线程读取的曲线数据
std::deque<Sample> samples;  // 以双端队列保存最近十秒的曲线采样数据
mjvFigure angle_figure, torque_figure;  // 分别保存角度响应图和力矩响应图的绘制数据
// 检测官方 Reset/历史回放导致的时间回退，避免沿用上一实验的积分。
const mjData* previous_data = nullptr;  // 保存上一物理步使用的状态对象地址，用于检测模型切换
double previous_time = -1;  // 保存上一步开始时的仿真时间，用于检测时间回退

void ClearHistory() {  // 清空动态曲线的全部历史样本
  const std::lock_guard<std::mutex> lock(samples_mutex);  // 在当前作用域持有曲线数据锁，防止并发读写采样队列
  samples.clear();  // 删除当前全部曲线样本
}  // 结束当前函数、命名空间或控制分支的作用域
void InitializeFigure(mjvFigure& figure, const char* title, const char* first) {  // 初始化图表样式、标题和首条曲线名称
  mjv_defaultFigure(&figure);  // 用 MuJoCo 默认值初始化图表结构
  figure.flg_legend = 1;  // 开启曲线图例显示
  figure.flg_extend = 0;  // 关闭坐标轴自动扩展，由后续代码显式设置范围
  figure.figurergba[3] = 0.95f;  // 将图表背景透明度设置为 0.95
  std::snprintf(figure.title, sizeof(figure.title), "%s", title);  // 将传入标题安全复制到图表标题数组
  std::snprintf(figure.xlabel, sizeof(figure.xlabel), "Time (s)");  // 设置横轴标签为仿真时间（秒）
  std::snprintf(figure.linename[0], sizeof(figure.linename[0]), "%s", first);  // 设置第一条曲线的图例名称
  figure.linergb[0][0] = 0.2f;  // 设置第一条曲线颜色的红色分量
  figure.linergb[0][1] = 0.8f;  // 设置第一条曲线颜色的绿色分量
  figure.linergb[0][2] = 1;  // 设置第一条曲线颜色的蓝色分量，组合得到亮蓝色
}  // 结束当前函数、命名空间或控制分支的作用域
void BuildUi(mujoco::Simulate* sim) {  // 在官方查看器左侧添加 PID 调参面板
  section = sim->ui0.nsect;  // 将当前左侧分区数量作为新 PID 分区的编号
  // 启动时折叠其他面板，让 PID 面板直接可见；原有官方功能仍可展开使用。
  for (int i = 0; i < section; ++i) sim->ui0.sect[i].state = 0;  // 逐个折叠原有面板，为 PID 参数区腾出可见空间
  mjuiDef items[] = {  // 定义由 MuJoCo 创建的 PID 面板控件列表
    {mjITEM_SECTION, "PID experiment", 1},  // 创建默认展开的 PID 实验分区
    {mjITEM_SLIDERNUM, "Kp", 1, &ui_parameters.kp, "0 200"},  // 比例增益滑块绑定 kp，允许范围为 0 至 200
    {mjITEM_SLIDERNUM, "Ki", 1, &ui_parameters.ki, "0 100"},  // 积分增益滑块绑定 ki，允许范围为 0 至 100
    {mjITEM_SLIDERNUM, "Kd", 1, &ui_parameters.kd, "0 40"},  // 微分增益滑块绑定 kd，允许范围为 0 至 40
    {mjITEM_SLIDERNUM, "Target (deg)", 1, &ui_parameters.target_deg, "-30 30"},  // 目标角度滑块，允许范围为 -30° 至 30°
    {mjITEM_SLIDERNUM, "Torque limit", 1, &ui_parameters.torque_limit, "0 20"},  // 力矩限幅滑块，允许范围为 0 至 20 Nm
    {mjITEM_SLIDERNUM, "Initial (deg)", 1, &ui_parameters.initial_deg, "-45 45"},  // 实验初始角度滑块，允许范围为 -45° 至 45°
    {mjITEM_BUTTON, "Clear integral", 1},  // 添加积分清零按钮，其控件编号为 6
    {mjITEM_BUTTON, "Reset experiment", 1},  // 添加实验重置按钮，其控件编号为 7
    {mjITEM_BUTTON, "Save parameters", 1},  // 添加参数保存按钮，其控件编号为 8
    {mjITEM_CHECKINT, "Response curves", 1, &show_curves},  // 添加绑定 show_curves 的曲线显示开关
    {mjITEM_STATIC, "Status", 1, nullptr, "Ready"},  // 添加状态文本，静态控件使用文本描述而非数据指针
    {mjITEM_END}  // 以结束标记终止控件定义列表
  };  // 结束结构体或初始化列表定义
  mjui_add(&sim->ui0, items);  // 将控件列表追加到官方查看器左侧面板
  std::snprintf(sim->ui0.sect[section].item[10].multi.name[0], mjMAXUINAME, "%s", status);  // 将当前提示文字写入编号为 10 的状态控件
}  // 结束当前函数、命名空间或控制分支的作用域
void HandleEvent(mujoco::Simulate* sim, mjuiItem* item) {  // 处理 PID 面板的滑块、按钮和显示开关事件
  if (item->sectionid != section) return;  // 忽略其他官方面板产生的事件
  // 与物理线程使用同一把锁，重置时不会与 mj_step 并发修改模型状态。
  const std::unique_lock<std::recursive_mutex> lock(sim->mtx);  // 持有查看器递归锁，使模型状态修改与渲染同步
  if (item->itemid <= 5) {  // 编号 0 至 5 对应六个参数滑块
    SetParameters(ui_parameters);  // 将界面最新参数副本提交到控制器
    std::snprintf(status, sizeof(status), "Parameters updated");  // 更新提示，表示新参数已经生效
  } else if (item->itemid == 6) {  // 处理积分清零按钮事件
    ResetIntegral();  // 清除上一阶段或上一实验的积分状态
    std::snprintf(status, sizeof(status), "Integral cleared");  // 提示误差积分已清零
  } else if (item->itemid == 7 && sim->m_ && sim->d_) {  // 仅在模型和状态有效时执行实验重置
    mj_resetData(sim->m_, sim->d_);  // 将仿真时间、速度和其他状态恢复为模型默认值
    SetInitialPose(sim->m_, sim->d_);  // 恢复指定初始倾角，同时清零控制积分
    previous_data = sim->d_;  // 记录重置后的状态对象地址
    previous_time = -1;  // 清除旧时间基准，允许从零时刻重新开始
    ClearHistory();  // 清空上一实验的响应曲线
    RecordResponse(sim->m_, sim->d_);  // 记录重置后的初始状态作为曲线起点
    sim->speed_changed = true;  // 要求物理循环重新对齐实际时间和仿真时间
    std::snprintf(status, sizeof(status), "Reset to initial angle");  // 提示实验已恢复设置的初始角度
  } else if (item->itemid == 8) {  // 处理保存参数按钮事件
    const bool ok = SaveParameters(PID_CONFIG_PATH);  // 保存参数至 CMake 指定的项目配置文件，并记录是否成功
    std::snprintf(status, sizeof(status), "%s", ok ? "Saved pid_parameters.txt" : "Save failed (see terminal)");  // 根据文件保存结果更新面板提示
    std::printf("%s: %s\n", ok ? "参数已保存" : "参数保存失败", PID_CONFIG_PATH);  // 在终端打印保存结果和配置文件路径
    std::fflush(stdout);  // 立即刷新标准输出缓冲，使终端实时显示信息
  }  // 结束当前函数、命名空间或控制分支的作用域
  std::snprintf(sim->ui0.sect[section].item[10].multi.name[0], mjMAXUINAME, "%s", status);  // 将当前提示文字写入编号为 10 的状态控件
  mjui_update(section, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());  // 刷新整个 PID 分区，使控件文字和数值立即更新
}  // 结束当前函数、命名空间或控制分支的作用域
void Plot(mujoco::Simulate* sim) {  // 将最近十秒样本转为 MuJoCo 图表并绘制
  if (!show_curves) return;  // 用户关闭曲线显示时跳过本帧绘图
  const std::lock_guard<std::mutex> lock(samples_mutex);  // 在当前作用域持有曲线数据锁，防止并发读写采样队列
  if (samples.empty()) return;  // 没有样本时不绘制空响应图
  const double end = std::max(10.0, samples.back().time);  // 横轴右端至少为 10 秒，之后跟随最新样本时间滚动
  const double begin = std::max(0.0, end - 10.0);  // 横轴左端为右端减十秒，且不小于零
  const int count = static_cast<int>(samples.size());  // 读取当前样本数量，转换为绘图库使用的整数类型
  double low = -1, high = 1, max_torque = 1;  // 初始化纵轴极值，避免曲线接近零时坐标范围退化
  for (int i = 0; i < count; ++i) {  // 遍历历史采样点，填写图表数据并统计纵轴范围
    const auto& s = samples[i];  // 以只读引用访问当前样本，避免复制
    low = std::min({low, s.angle, s.target});  // 更新实际角度和目标角度的最小值
    high = std::max({high, s.angle, s.target});  // 更新实际角度和目标角度的最大值
    max_torque = std::max(max_torque, std::abs(s.torque));  // 更新控制力矩的最大绝对值
    angle_figure.linedata[0][2*i] = angle_figure.linedata[1][2*i] = s.time;  // 为实际角度曲线和目标曲线写入相同的时间坐标
    angle_figure.linedata[0][2*i+1] = s.angle;  // 写入实际角度曲线的纵坐标
    angle_figure.linedata[1][2*i+1] = s.target;  // 写入目标角度曲线的纵坐标
    torque_figure.linedata[0][2*i] = s.time;  // 写入力矩曲线的时间坐标
    torque_figure.linedata[0][2*i+1] = s.torque;  // 写入力矩曲线的纵坐标
  }  // 结束当前函数、命名空间或控制分支的作用域
  angle_figure.linepnt[0] = angle_figure.linepnt[1] = torque_figure.linepnt[0] = count;  // 将三条曲线的有效点数都设为当前样本数量
  for (mjvFigure* f : {&angle_figure, &torque_figure}) {  // 统一设置角度图和力矩图的横轴范围
    f->range[0][0] = begin;  // 设置横轴起始时间
    f->range[0][1] = end;  // 设置横轴结束时间
  }  // 结束当前函数、命名空间或控制分支的作用域
  const double margin = (high-low)*0.15;  // 按角度数据跨度的 15% 计算上下留白
  angle_figure.range[1][0] = low-margin;  // 设置角度图纵轴下界，并保留底部空间
  angle_figure.range[1][1] = high+margin;  // 设置角度图纵轴上界，并保留顶部空间
  torque_figure.range[1][0] = -max_torque*1.15;  // 设置对称的力矩纵轴负向范围，并留出 15% 余量
  torque_figure.range[1][1] = max_torque*1.15;  // 设置对称的力矩纵轴正向范围，并留出 15% 余量
  const mjrRect view = sim->uistate.rect[3];  // 读取不包含左右控制面板的三维视图矩形
  // 图表并排放在三维视图底部，占约 1/3 高度，不遮挡两侧控制面板。
  const int height = std::max(1, view.height / 3);  // 将响应图高度设为视图高度的三分之一，至少为一个像素
  const int width = view.width / 2;  // 将三维视图宽度平分给两幅响应图
  if (width <= 0) return;  // 视图过窄或最小化时跳过无效尺寸的绘制
  mjr_figure({view.left, view.bottom, width, height}, &angle_figure, &sim->platform_ui->mjr_context());  // 在三维视图底部左半区绘制角度响应图
  mjr_figure({view.left+width, view.bottom, view.width-width, height}, &torque_figure, &sim->platform_ui->mjr_context());  // 在三维视图底部右半区绘制控制力矩图
}  // 结束当前函数、命名空间或控制分支的作用域
}  // namespace  // 结束当前函数、命名空间或控制分支的作用域

void BeforePhysicsStep(const mjModel* m, const mjData* d) {  // 在物理步前检测实验重置并更新积分
  if (d != previous_data || d->time <= previous_time) {  // 状态对象改变或时间未递增时，视为新实验或历史回退
    ResetIntegral();  // 清除上一阶段或上一实验的积分状态
    ClearHistory();  // 清空上一实验的响应曲线
  }  // 结束当前函数、命名空间或控制分支的作用域
  previous_data = d;  // 保存当前状态对象，供下一步比较
  previous_time = d->time;  // 保存本次物理步开始时的仿真时间
  AdvanceIntegral(m, d);  // 按模型步长更新一次误差积分
}  // 结束当前函数、命名空间或控制分支的作用域
void RecordResponse(const mjModel* m, const mjData* d) {  // 按采样间隔记录角度、目标及控制力矩
  const int joint = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");  // 按 XML 名称查找倒立摆关节编号
  const int motor = mj_name2id(m, mjOBJ_ACTUATOR, "pendulum_motor");  // 按 XML 名称查找倒立摆电机执行器编号
  if (joint < 0 || motor < 0) return;  // 模型缺少指定对象时跳过本次处理，避免无效下标
  const auto p = GetParameters();  // 读取一份线程安全的当前参数快照
  const std::lock_guard<std::mutex> lock(samples_mutex);  // 在当前作用域持有曲线数据锁，防止并发读写采样队列
  if (!samples.empty() && d->time < samples.back().time) samples.clear();  // 发现时间回退时丢弃旧实验样本
  // 100Hz 采样并保留最近 10 秒，限制内存和绘制开销。
  if (!samples.empty() && d->time - samples.back().time < 0.01) return;  // 距上次采样不足 10ms 时跳过，以控制采样频率
  samples.push_back({d->time, d->qpos[m->jnt_qposadr[joint]]*180/mjPI,  // 添加当前时间及由 rad 转为 ° 的实际角度
                     p.target_deg, d->ctrl[motor]});  // 继续填入当前目标角度和执行器控制力矩
  while (samples.size() > mjMAXLINEPNT || samples.front().time < d->time-10) samples.pop_front();  // 持续删除过量或超过十秒窗口的最旧样本
}  // 结束当前函数、命名空间或控制分支的作用域
void InstallDashboard(mujoco::Simulate& sim) {  // 加载保存参数并注册面板、事件及曲线渲染回调
  // 初次启动无保存文件时使用默认参数；下次启动自动恢复保存的参数。
  if (LoadParameters(PID_CONFIG_PATH)) std::snprintf(status, sizeof(status), "Saved parameters loaded");  // 若成功加载保存参数，则更新面板状态提示
  ui_parameters = GetParameters();  // 将当前控制参数复制到滑块绑定的数据对象
  InitializeFigure(angle_figure, "Angle response (deg)", "Actual");  // 初始化角度响应图及实际角度曲线
  std::snprintf(angle_figure.linename[1], sizeof(angle_figure.linename[1]), "Target");  // 为第二条角度曲线设置目标角度图例
  angle_figure.linergb[1][0] = 1;  // 设置目标曲线的红色分量
  angle_figure.linergb[1][1] = 0.5f;  // 设置目标曲线的绿色分量
  angle_figure.linergb[1][2] = 0.2f;  // 设置目标曲线的蓝色分量，组合为橙色
  InitializeFigure(torque_figure, "Control torque (Nm)", "Torque");  // 初始化力矩响应图及力矩曲线
  sim.custom_ui = BuildUi;  // 注册 PID 面板创建回调
  sim.custom_event = HandleEvent;  // 注册 PID 面板事件处理回调
  sim.custom_render = Plot;  // 注册每帧响应曲线绘制回调
}  // 结束当前函数、命名空间或控制分支的作用域
}  // namespace pendulum  // 结束当前函数、命名空间或控制分支的作用域
