# 单关节机械臂 PID 仿真（C++）

使用与 `1-ball_test` 相同的 MuJoCo 官方 C++ 查看器，显示完整控制面板。
保留 `motor.xml` 机械臂及 PID 参数：Kp=5、Ki=0.5、Kd=1，目标 90°，力矩限制 ±5 Nm。
终端每 0.1 秒仿真时间输出角度、误差和力矩，不再弹出 Matplotlib 窗口。

与 `1-ball_test` 一样，使用 C++17 和 CMake，在系统终端直接编译运行：

```bash
cmake -S . -B build
cmake --build build --parallel
./build/motor_simulate
```

也可在系统终端执行 `./run.sh`，自动编译并启动同一查看器。

空格暂停/继续，暂停时停止输出；使用面板 Reset 重置，PID 积分随之清零。
窗口持续运行到手动关闭。可通过 File 面板加载模型，或传入 XML 路径。
程序可从任意工作目录启动。依赖 CMake、C++17、MuJoCo SDK 和 GLFW，
构建时查找 `~/.mujoco` 中的 SDK；GLFW 优先使用系统库，也可使用
`third_party/glfw/lib` 中保留的原生动态库（许可证见 `third_party/GLFW/LICENSE.md`）。

无窗口验证运行 10 秒：

```bash
./build/motor_simulate --headless
```

`third_party` 中保留了参考项目的 MuJoCo 官方查看器源码及许可证。

验证说明：保留的原模型和 PID 参数在 10 秒时角度约为 31.16°，尚未收敛到 90°。
无窗口验证检查数值稳定性，不以达到目标角度作为通过条件。

工程文件：`main.cpp` 包含官方查看器主程序、PID 控制和终端输出；
`motor.xml` 定义机械臂；`CMakeLists.txt` 配置依赖和编译。
