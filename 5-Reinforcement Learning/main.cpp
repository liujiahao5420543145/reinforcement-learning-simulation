// 无窗口仿真入口，对应 CMake 目标 car_test。
// 官方图形界面的入口位于 simulate_main.cpp，对应 car_simulate。
#include "car_controller.h"  // 小车 PPO 控制器及其使用的 MuJoCo 接口
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

// argc 为命令行参数数量；argv 保存程序名及各项参数。
int main(int argc, char** argv) {
  // 默认运行 12 个仿真秒；模型和策略路径由 CMake 编译定义提供。
  double duration = 12;
  const char* xml = DEFAULT_MODEL_PATH;
  const char* policy = DEFAULT_POLICY_PATH;
  std::string csv;  // 为空时不导出数据，指定 --csv 后写入对应文件。
  try {
    // 解析命令行，跳过 argv[0] 中的程序名。
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "car_test [--headless] [--duration 秒] [--csv 路径] [--policy 参数文件] [模型.xml]\n默认图形界面请运行 car_simulate\n";
        return 0;
      }
      // 此程序始终无窗口运行；接受该标志以便显式表达运行模式。
      if (arg == "--headless") continue;
      if (arg == "--duration" || arg == "--csv" || arg == "--policy") {
        // 这三种选项均需读取紧随其后的参数值。
        if (++i == argc) throw std::runtime_error("选项缺少参数");
        if (arg == "--duration") duration = std::stod(argv[i]);
        else if (arg == "--csv") csv = argv[i];
        else policy = argv[i];
      } else if (arg.rfind("--", 0) == 0) throw std::runtime_error("未知参数: " + arg);
      else xml = argv[i];  // 普通位置参数作为待加载的 XML 模型路径。
    }
    // 拒绝 NaN、无穷大和非正数，保证仿真时长有效。
    if (!std::isfinite(duration) || duration <= 0) throw std::runtime_error("时长必须为正数");
    // 加载并编译 XML；失败时 MuJoCo 将错误原因写入 error。
    // unique_ptr 配合 MuJoCo 删除函数，在正常退出或异常时自动释放资源。
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(mj_loadXML(xml, nullptr, error, sizeof(error)), mj_deleteModel);
    if (!m) throw std::runtime_error(error);
    // 控制器依赖这两个命名对象，提前检查以避免使用无效索引。
    if (mj_name2id(m.get(), mjOBJ_JOINT, "car_slide") < 0 || mj_name2id(m.get(), mjOBJ_ACTUATOR, "car_motor") < 0)
      throw std::runtime_error("模型缺少 car_slide/car_motor");
    // mjModel 保存模型参数，mjData 保存时间、位置、速度及控制输入等动态状态。
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    if (!d) throw std::runtime_error("创建仿真数据失败");
    // 读取导出的 PPO 网络参数，并绑定模型、初始化小车状态。
    CarController controller; controller.load(policy); controller.bind(m.get(), d.get());
    // 可选 CSV 输出：记录各物理步结束后的状态及该步使用的控制指令。
    std::ofstream file;
    if (!csv.empty()) {
      file.open(csv);
      if (!file) throw std::runtime_error("无法写入 CSV");
      file << "time,target_velocity,actual_velocity,position,force\n" << std::setprecision(12);
    }
    // 关节编号不一定等于状态数组下标，需通过地址数组定位。
    int joint = mj_name2id(m.get(), mjOBJ_JOINT, "car_slide");
    // v 索引速度数组 qvel，q 索引位置数组 qpos。
    int v = m->jnt_dofadr[joint], q = m->jnt_qposadr[joint];
    // 累加速度误差平方，结束后计算均方根误差（RMSE）。
    double squared_error = 0; size_t count = 0;
    // 按仿真时间运行，不等待真实时间；微小容差用于处理浮点累计误差。
    while (d->time + 1e-9 < duration) {
      // 每次推进一个物理步；控制器内部每 10 个物理步更新一次 PPO 动作，
      // 并负责带噪声观测、分段目标速度和终端状态输出。
      controller.step(m.get(), d.get());
      // 速度非有限值或幅值超过 5 m/s 时停止，沿用原训练环境的失败阈值。
      if (!std::isfinite(d->qvel[v]) || std::abs(d->qvel[v]) > 5) throw std::runtime_error("速度异常");
      squared_error += std::pow(controller.target() - d->qvel[v], 2); ++count;
      // 列顺序与 CSV 表头一致：时间、目标速度、实际速度、位置、驱动力。
      if (file.is_open()) file << d->time << ',' << controller.target() << ',' << d->qvel[v] << ',' << d->qpos[q] << ',' << controller.force() << '\n';
    }
    // RMSE = sqrt(误差平方和 / 采样数)，单位为 m/s。
    std::cout << "速度跟踪 RMSE: " << std::sqrt(squared_error / count) << " m/s\n";
    return 0;
  // 统一报告参数、模型、策略或仿真异常，以非零退出码表示失败。
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
