# 倒立摆实时 PID 实验

采用 C++17、CMake 和 MuJoCo 官方 simulate 查看器框架。
在同一窗口中调节 PID 滑块、观察倒立摆及实时动态响应曲线。

## 编译运行

```bash
cmake -S . -B build
cmake --build build -j 2
./build/pendulum_simulate
```

依赖 CMake、C++17 编译器和独立 MuJoCo SDK（自动查找 `~/.mujoco/mujoco-*`，
也可通过 `-DMUJOCO_ROOT=/path/to/mujoco` 指定）。优先使用系统 GLFW，
项目也提供 Linux x86_64 GLFW 共享库于 `third_party/GLFW/lib`。
模型使用构建目录中的绝对路径，可从任意目录启动，也可传入其他 XML 路径。

## 实时实验

左侧 **PID experiment** 面板默认展开，其余官方面板可自行展开。
界面标签使用英文以适配 MuJoCo 内置字体。

| 控件 | 功能与范围 |
| --- | --- |
| Kp / Ki / Kd | 比例、积分、微分增益；范围 0–200 / 0–100 / 0–40 |
| Target (deg) | 目标角度，−30° 至 30° |
| Torque limit | 控制力矩限幅，0–20 Nm，同时遵守 XML 执行器范围 |
| Initial (deg) | 下次重置实验使用的初始倾角，−45° 至 45° |
| Clear integral | 立即清零积分，保留当前物理状态和曲线 |
| Reset experiment | 时间归零、速度清零，恢复指定初始倾角，清空积分和曲线 |
| Save parameters | 将当前六个参数保存至项目根目录 `pid_parameters.txt` |
| Response curves | 显示或隐藏动态响应曲线 |

滑块修改立即生效，初始角度在重置实验时生效。改变参数时积分继续保留，
需要比较不同参数的响应时，调好参数后按 **Reset experiment** 开始新实验。
下次启动会自动加载保存的参数；文件缺失或格式无效时使用默认值。
参数文件按顺序保存 Kp、Ki、Kd、目标角度、力矩限幅、初始角度。
默认值为 `30 0 8 0 20 10`，Ki=0 时等同于原 PD 控制。

三维视图底部并排显示角度响应（实际/目标，单位 °）和控制力矩（Nm），
约以 100Hz 采样，滚动显示最近 10 秒，纵轴自动缩放。
系统终端约每 20ms 输出时间、角度、角速度和力矩。
空格暂停/继续；暂停时不推进时间、不累积积分、不持续输出遥测。
窗口持续运行至手动关闭。官方 Reset 仍恢复 XML 默认的竖直姿态；
要以相同初始倾角重复实验，请使用 **Reset experiment**。

控制器采用 `tau = Kp * error + Ki * integral - Kd * velocity`，内部使用 rad。
微分作用于测量速度，避免目标角度跳变引发微分冲击。
积分只在物理步前更新，不在 MuJoCo 回调中累积，避免暂停前向计算和 RK4
多次回调重复积分。输出饱和且误差使饱和加剧时，停止积分累积。

## 文件职责

- `main.cpp`：程序入口、模型路径选择。
- `simulation.h/.cpp`：模型加载、物理线程、官方查看器及运行循环。
- `pendulum_control.h/.cpp`：PID 参数、控制力矩、积分与参数持久化。
- `pid_dashboard.h/.cpp`：滑块、按钮、实验重置及动态曲线。
- `telemetry.h/.cpp`：终端实时数据输出。
- `pendulum.xml`：倒立摆物理模型。
- `third_party`：官方查看器源码、依赖及许可证；查看器增加了三个应用扩展回调。
- `tests/pid_control_test.cpp`：无窗口控制器回归验证。

```bash
ctest --test-dir build --output-on-failure
```

## 实验二：外部扰动测试

启动或点击 **Reset experiment** 后，默认在仿真第 5 秒对
`pendulum_joint` 施加 **2 Nm、持续 0.2 秒** 的广义外力矩。
通过 `data.qfrc_applied[model.jnt_dofadr[joint]]` 写入力矩，脉冲结束后显式清零。
控制器继续通过 `ctrl` 输出回正力矩，两者独立作用。

- **Auto disturbance**：启用每次实验一次的自动扰动。
- **Pulse at (s)**：自动触发的仿真时刻，0–30 秒。
- **Pulse length (s)**：脉冲持续时间，0.02–1 秒。
- **External (Nm)**：外部力矩，−10 至 10 Nm，正负表示相反方向。
- **Apply pulse now**：立即触发一次脉冲，运行中可重复点击；暂停时等待继续仿真。

力矩图的 **Torque / External** 分别显示控制力矩和外部力矩。
重置会取消当前脉冲并重新准备自动扰动；暂停不消耗脉冲的仿真时间。
扰动参数只在本次运行中有效，保存按钮仍只保存 PID 参数。
保持 Target 为 0°，观察扰动后的角度峰值与回零过程。
默认 PID 的无窗口回归测试中，2 Nm / 0.2 秒脉冲引起约 1.804° 峰值偏转，
随后恢复竖直；更大的扰动或不同 PID 参数可能导致失稳。
