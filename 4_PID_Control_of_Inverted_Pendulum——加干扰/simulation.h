#pragma once  // 确保同一个头文件在每个编译单元中只展开一次

/**
 * 启动仿真：主线程负责界面，后台线程负责物理计算。
 * @param model_path 模型文件路径，在整个运行期间保持有效。
 * @return 正常关闭窗口并回收物理线程后返回 0。
 */
int RunSimulation(const char* model_path);  // 创建官方查看器并管理物理线程，直到用户关闭窗口，此处为对外声明
