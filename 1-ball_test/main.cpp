#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char* argv[]) {
    bool headless = false;
    std::filesystem::path xml_path = DEFAULT_MODEL_PATH;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") {
            headless = true;
        } else if (arg == "--help") {
            std::cout << "用法: ball_test [模型.xml] [--headless]\n";
            return 0;
        } else if (arg.rfind("--", 0) == 0) {
            std::cerr << "未知参数: " << arg << '\n';
            return 1;
        } else {
            xml_path = arg;
        }
    }

    char error[1024] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(xml_path.string().c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) {
        std::cerr << "加载模型失败: " << error << '\n';
        return 1;
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    if (!data) {
        std::cerr << "创建仿真数据失败\n";
        return 1;
    }
    const int ball_id = mj_name2id(model.get(), mjOBJ_BODY, "ball");
    if (ball_id < 0) {
        std::cerr << "模型中未找到名为 ball 的刚体\n";
        return 1;
    }

    GLFWwindow* window = nullptr;
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    if (!headless) {
        glfwSetErrorCallback([](int, const char* description) {
            std::cerr << "GLFW: " << description << '\n';
        });
        if (!glfwInit()) {
            std::cerr << "无法初始化窗口；可使用 --headless 运行物理仿真\n";
            return 1;
        }
        window = glfwCreateWindow(1200, 900, "MuJoCo - Ball Fall", nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            return 1;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
            if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            }
        });
        camera.lookat[2] = 1.0;
        camera.distance = 5.0;
        camera.azimuth = 90.0;
        camera.elevation = -20.0;
        mjv_makeScene(model.get(), &scene, 2000);
        mjr_makeContext(model.get(), &context, mjFONTSCALE_150);
    }

    mj_forward(model.get(), data.get());
    const auto start = std::chrono::steady_clock::now();
    auto next_frame = start;
    std::cout << std::fixed << std::setprecision(3);
    while (data->time < 10.0 && (!window || !glfwWindowShouldClose(window))) {
        mj_step(model.get(), data.get());
        // 更新派生位置，使输出与当前仿真时间一致。
        mj_forward(model.get(), data.get());
        std::cout << "时间: " << data->time << "s 高度: "
                  << data->xpos[3 * ball_id + 2] << "m\n";

        if (window) {
            const auto now = std::chrono::steady_clock::now();
            // 物理步长为 2ms，画面独立以约 60Hz 更新。
            if (now >= next_frame || data->time >= 10.0) {
                mjrRect viewport{0, 0, 0, 0};
                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
                mjv_updateScene(model.get(), data.get(), &option, nullptr, &camera,
                                mjCAT_ALL, &scene);
                mjr_render(viewport, &scene, &context);
                glfwSwapBuffers(window);
                next_frame = now + std::chrono::milliseconds(16);
            }
            glfwPollEvents();
            std::this_thread::sleep_until(start + std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(std::chrono::duration<double>(data->time)));
        }
    }

    if (window) {
        mjr_freeContext(&context);
        mjv_freeScene(&scene);
        glfwDestroyWindow(window);
        glfwTerminate();
    }
    return 0;
}
