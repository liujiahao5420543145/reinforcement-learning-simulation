# C++ PPO 小车速度跟踪

沿用 `1-ball_test` 的 C++17、CMake 和 MuJoCo 官方 Simulate 框架，保留本项目的小车模型及已训练的 PPO 控制器。

```bash
cmake -S . -B build
cmake --build build -j
./build/car_simulate
```

默认显示官方控制面板和三维小车，终端每 0.2 仿真秒输出时间、目标速度、实际速度、位置及驱动力。空格暂停/继续，Backspace 重置，也可使用面板操作。暂停时停止终端输出，重置后时间从零开始。窗口持续运行，直到手动关闭。

目标速度每 12 秒循环：0–3 秒为 1 m/s，3–6 秒为 0.5 m/s，6–9 秒为 -1 m/s，9–12 秒为 0 m/s。可持续观察。可在官方相机面板选择跟踪 car，避免长时间运行后小车离开视野。

## 文件

- `simulate_main.cpp`：官方界面、渲染主线程及物理后台线程。
- `car_controller.h`：C++ PPO 前向推理、带噪声速度观测、20 ms 控制周期及重置处理。
- `models/ppo_car.txt`：从已有 `ppo_car.zip` 导出的策略网络参数；运行无需 Python、PyTorch 或 SB3。
- `main.cpp`：无窗口验证及 CSV 数据导出入口 `car_test`。
- `third_party`：复制自参考项目的官方查看器、GLFW 头文件及 lodepng，保留原有许可证。

```bash
./build/car_test --headless --duration 24 --csv /tmp/car_tracking.csv
```

CSV 包含时间、目标速度、实际速度、位置和驱动力，可用于后续绘图。

## Python 训练

仅保留训练及参数导出所需的 Python 内容：`train.py`、`car_env.py`、`tools/export_policy.py`，以及训练模型 `models/ppo_car.zip`、训练日志 `logs/` 和依赖环境 `.venv/`。旧 Python 测试入口 `test.py` 和生成的字节码缓存已删除。

`.venv/` 同时提供当前 C++ 构建使用的 MuJoCo 头文件和 GLFW 动态库，应保留以保证编译及窗口运行。C++ 程序执行时不调用 Python 解释器。

训练仍使用原 Python 实现：

```bash
.venv/bin/python train.py
```

重新训练后，更新 C++ 参数：

```bash
.venv/bin/python tools/export_policy.py
cmake -S . -B build
```

C++ 随机数生成器与 NumPy 不同，所以即使都使用种子 42，噪声及随机初始条件也不逐项一致；同一观测下策略输出一致。

CMake 自动发现当前 `.venv` 或用户 MuJoCo SDK。也可指定 `-DMUJOCO_ROOT=/path/to/mujoco`。查看器源码来自参考项目的 MuJoCo 3.17.0，需使用兼容的 SDK。
