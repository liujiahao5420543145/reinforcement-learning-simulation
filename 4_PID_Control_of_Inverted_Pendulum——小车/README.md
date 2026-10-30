# 实验三：小车倒立摆

基于 C++17、MuJoCo 和官方 simulate 查看器。控制小车的水平驱动力，使上方摆杆保持竖直，并使小车回到轨道中心。保留实时 PID 调参、实验重置、参数保存和响应曲线。

## 编译运行

```bash
cmake -S . -B build-cart
cmake --build build-cart -j 2
./build-cart/pendulum_simulate
ctest --test-dir build-cart --output-on-failure
```

自动查找 `~/.mujoco/mujoco-*`，也可指定 `-DMUJOCO_ROOT=/path/to/mujoco`。优先使用系统 GLFW，附带 Linux x86_64 GLFW 库。可向程序传入其他 XML 路径。

## 模型与控制

`pendulum.xml` 包含两个耦合自由度：小车 `cart_slide` 沿 x 轴平移，摆杆 `pendulum_joint` 绕 y 轴旋转。摆杆作为小车的子 body，随小车移动。小车质量 1 kg，均匀摆杆质量 1 kg、长度 1 m。轨道关节范围为 ±2.4 m，端部为软限位；导轨和端块仅用于显示。小车由滑动关节约束，未增加轮子自由度。

唯一执行器 `cart_motor` 作用于小车，gear=1，控制量为水平力 N，限幅 ±20 N。摆杆转轴无电机。零角度为竖直向上，正角度向 +x 倾斜，因此需要正向驱动小车接住摆杆。

控制器内部使用 rad、m、s：

```text
correction = clip(0.04*x + 0.06*x_dot, -0.15, 0.15)
error = theta - target + correction
F = clip(Kp*error + Ki*integral + Kd*theta_dot, -limit, limit)
```

位置与速度外环为摆杆产生回中倾角修正，角度 PID 内环通过水平力平衡摆杆。外环增益固定在 `pendulum_control.cpp`。这是局部直立平衡控制，不包含从下垂状态起摆。默认参数经过 ±10° 初始倾角及小车位置、速度偏移的动力学测试；大角度、低驱动力或不合适的增益可能使摆杆倒下。

## 实时实验

左侧 **PID experiment** 面板调节 Kp/Ki/Kd（0–200 / 0–100 / 0–40）、Target（±30°）、Force limit（0–20 N）及 Initial（±45°）。默认参数为 `100 0 20 0 20 10`。初始角度仅在重置时生效。保持中心平衡时应将 Target 设为 0；非零目标角度会与回中外环相互作用，不能作为任意恒定倾角跟踪实验。

- **Clear integral**：清零积分。
- **Reset experiment**：时间归零、小车回中心、速度清零，恢复初始倾角并清空曲线。
- **Save parameters**：将六个参数按面板顺序保存到 `pid_parameters.txt`，启动时自动加载。
- **Response curves**：显示最近 10 秒角度（实际/设定值）及小车驱动力。角度设定值曲线不包含外环倾角修正。

终端每约 20 ms 输出角度、角速度、小车位置、速度和水平驱动力。空格暂停/继续。积分仅在物理步前更新，避免暂停或 RK4 回调重复累积，并采用条件积分抗饱和。

## 代码位置

- `pendulum_control.h/.cpp`：控制、初始状态和参数持久化。
- `pendulum.xml`：小车、摆杆、导轨及执行器。
- `pid_dashboard.h/.cpp`：实时面板、实验重置和响应曲线。
- `telemetry.h/.cpp`：终端状态输出。
- `simulation.cpp`、`main.cpp`：模型加载与查看器运行。
- `tests/pid_control_test.cpp`：平衡收敛、双自由度回中、限幅、积分和参数读写验证。

旧固定底座实验的参数与当前系统控制方向和动力学不同，应使用本工程随附的新参数。
