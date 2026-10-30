#include "simulation.h"  // 引入仿真启动接口声明

// 程序入口：选择 XML 模型，然后启动官方查看器和物理仿真。
// 不传参数时使用 CMake 指定的模型绝对路径；argv[1] 可指定其他模型。
int main(int argc, char* argv[]) {  // 程序入口，负责选择模型并启动仿真；测试文件中用于执行回归验证
    const char* model_path = argc > 1 ? argv[1] : DEFAULT_MODEL_PATH;  // 有命令行参数时使用用户模型路径，否则使用 CMake 提供的默认路径
    return RunSimulation(model_path);  // 启动查看器和仿真，并将运行结果作为进程返回值
}  // 结束当前函数、命名空间或控制分支的作用域
