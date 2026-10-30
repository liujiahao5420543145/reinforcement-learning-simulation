# MuJoCo 单关节机械臂

默认运行方式与 `1-ball_test` 的 `ball_simulate` 一致：显示 MuJoCo 官方控制面板，在终端输出仿真时间、目标角度和当前角度。保留机械臂模型，目标角度为 45°。

在系统终端中执行：

```bash
./run.sh
```

或手动编译运行：

```bash
cmake -S . -B build
cmake --build build -j
./build/mujoco_joint
```

空格暂停或继续；暂停时停止输出，重置后时间重新计数。窗口持续运行，直到手动关闭。面板支持相机操作、模型重载和仿真参数调整。输出间隔为 0.02 秒仿真时间。

`main.cpp` 是官方面板和物理控制的统一入口，`arm.xml` 是机械臂模型。可通过 `./run.sh /path/to/model.xml` 加载其他模型；只有包含 `arm_position` 执行器和 `arm_hinge` 关节的模型才支持对应的目标控制与角度输出。
