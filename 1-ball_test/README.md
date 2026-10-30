# MuJoCo 小球下落（C++）

## 默认运行约定

以后默认使用 `ball_simulate`，在系统终端中运行，同时显示 MuJoCo 官方控制面板并打印时间和小球高度。空格暂停或继续，窗口持续运行至手动关闭。后续工程修改应保留此行为，详见 `AGENTS.md`。

```bash
cd ~/MuJoCo_Projects/ball_test
cmake -S . -B build &&
cmake --build build -j &&
./build/ball_simulate
```

使用 C++17、MuJoCo 和 GLFW。运行 10 秒仿真，逐步输出时间和小球高度；窗口按约 60Hz 刷新，按 Esc 或关闭窗口可提前退出。`ball.xml` 保留原有物理设置。

## 编译和运行

需要 CMake ≥ 3.16、C++17 编译器、MuJoCo SDK 和 GLFW。
本工程会自动查找系统库、项目 `.venv`、`~/.mujoco/mujoco-*` SDK 和 `~/mujoco-env` 中的动态库。编译和运行不需要启动 Python，但需要保留所使用的动态库目录。旧依赖目录被移除后，重新运行 CMake 会自动查找可用路径。

```bash
cmake -S . -B build
cmake --build build -j
./build/ball_test
```

也可以使用独立安装的 MuJoCo SDK 和系统 GLFW（Ubuntu/Debian 上安装 `libglfw3-dev`）：

```bash
cmake -S . -B build -DMUJOCO_ROOT=/path/to/mujoco
cmake --build build -j
```

CMake 会将模型复制到构建目录，默认加载该模型，因此可从其他工作目录启动。修改 XML 后重新运行构建，或直接指定模型路径：

```bash
./build/ball_test /path/to/ball.xml
```

无窗口模式无需显示服务，以最快速度运行相同的 10 秒物理仿真：

```bash
./build/ball_test --headless
```

## 文件

- `main.cpp`：模型加载、物理仿真、终端输出与 GLFW 可视化。
- `ball.xml`：地面、重力和自由下落小球。
- `CMakeLists.txt`：依赖查找与构建配置。
- `third_party/GLFW`：GLFW 3.4 官方头文件及其许可证。

窗口提供固定视角和 Esc 退出，不包含 Python passive viewer 的交互面板。

## 使用 MuJoCo 官方控制面板

要同时显示官方控制面板并在终端打印时间和小球高度，运行本工程新增的 C++ 查看器：

```bash
cmake -S . -B build
cmake --build build -j
./build/ball_simulate
```

`ball_simulate` 基于 MuJoCo 3.17.0 官方查看器源码，每个物理步输出一次当前小球高度，暂停时停止输出，重置后时间重新计数。窗口持续运行直到手动关闭，不受 10 秒限制。可以传入其他 XML 路径；模型没有 `ball` 刚体时不输出高度。

需要 File、Option、Simulation、Watch 等面板时，使用 SDK 自带的 `simulate` 查看器加载模型：

```bash
cd ~/MuJoCo_Projects/ball_test
~/.mujoco/mujoco-3.17.0/bin/simulate "$PWD/ball.xml"
```

该命令运行官方查看器并直接加载 XML，不执行 `main.cpp` 中的高度输出和 10 秒退出逻辑。空格可暂停或继续仿真。
