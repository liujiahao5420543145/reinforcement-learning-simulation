#pragma once
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <stdexcept>
#include <vector>

// Deterministic SB3 MlpPolicy: [noisy velocity, target] -> tanh MLP -> clipped action.
class CarController {
 public:
  struct Layer { int in, out; std::vector<float> weights, bias; };
  void load(const char* path) {
    std::ifstream f(path);
    int count = 0;
    if (!(f >> count) || count != 3) throw std::runtime_error("无法读取 PPO 参数文件");
    layers_.clear();
    const int dims[] = {2, 64, 64, 1};
    for (int n = 0; n < count; ++n) {
      Layer layer;
      if (!(f >> layer.in >> layer.out) || layer.in != dims[n] || layer.out != dims[n+1])
        throw std::runtime_error("PPO 网络维度错误");
      layer.weights.resize(layer.in * layer.out); layer.bias.resize(layer.out);
      for (auto& v : layer.weights) if (!(f >> v) || !std::isfinite(v)) throw std::runtime_error("PPO 权重错误");
      for (auto& v : layer.bias) if (!(f >> v) || !std::isfinite(v)) throw std::runtime_error("PPO 偏置错误");
      layers_.push_back(std::move(layer));
    }
  }
  float predict(float velocity, float target) const {
    std::vector<float> x{velocity, target};
    for (size_t n = 0; n < layers_.size(); ++n) {
      const auto& layer = layers_[n];
      std::vector<float> y(layer.out);
      for (int i = 0; i < layer.out; ++i) {
        float v = layer.bias[i];
        for (int j = 0; j < layer.in; ++j) v += layer.weights[i*layer.in+j] * x[j];
        y[i] = n+1 == layers_.size() ? v : std::tanh(v);
      }
      x = std::move(y);
    }
    return std::clamp(x.at(0), -1.0f, 1.0f);
  }
  void bind(mjModel* m, mjData* d) {
    int joint = mj_name2id(m, mjOBJ_JOINT, "car_slide");
    motor_ = mj_name2id(m, mjOBJ_ACTUATOR, "car_motor");
    valid_ = joint >= 0 && motor_ >= 0;
    if (!valid_) { std::fprintf(stderr, "模型缺少 car_slide/car_motor，PPO 控制已禁用\n"); return; }
    qpos_ = m->jnt_qposadr[joint]; qvel_ = m->jnt_dofadr[joint];
    reset(m, d);
  }
  static double targetAt(double t) {
    double phase = std::fmod(t, 12.0);
    return phase < 3 ? 1 : phase < 6 ? 0.5 : phase < 9 ? -1 : 0;
  }
  void step(mjModel* m, mjData* d, bool output = true) {
    if (!valid_) { mj_step(m, d); return; }
    // Official viewer reset/history rewind changes simulation time.
    if (d->time < previous_time_ || (d->time == 0 && previous_time_ > 0)) reset(m, d);
    if (d->time + 1e-9 >= next_control_) {
      target_ = targetAt(d->time);
      force_ = 10.0 * predict(static_cast<float>(d->qvel[qvel_] + noise_(rng_)), static_cast<float>(target_));
      next_control_ = d->time + 10 * m->opt.timestep;
    }
    d->ctrl[motor_] = force_;
    mj_step(m, d);
    previous_time_ = d->time;
    if (output && d->time + 1e-9 >= next_print_) {
      std::printf("时间: %.3fs | 目标: %.2fm/s | 实际: %.3fm/s | 位置: %.3fm | 力: %.3fN\n",
                  d->time, target_, d->qvel[qvel_], d->qpos[qpos_], force_);
      std::fflush(stdout); next_print_ = d->time + 0.2;
    }
  }
  double target() const { return target_; }
  double force() const { return force_; }
 private:
  void reset(mjModel* m, mjData* d) {
    rng_.seed(42); noise_.reset();
    d->qvel[qvel_] = std::uniform_real_distribution<double>(-0.3, 0.3)(rng_);
    // Match the original reset's target sampling before damping sampling.
    (void)std::uniform_real_distribution<double>(-1.5, 1.5)(rng_);
    m->dof_damping[qvel_] = std::uniform_real_distribution<double>(0.5, 2.0)(rng_);
    previous_time_ = d->time; next_control_ = d->time; next_print_ = d->time;
    target_ = targetAt(d->time); force_ = 0;
    mj_forward(m, d);
  }
  std::vector<Layer> layers_;
  std::mt19937 rng_{42};
  std::normal_distribution<double> noise_{0, 0.1};
  bool valid_ = false;
  int qpos_ = 0, qvel_ = 0, motor_ = 0;
  double previous_time_ = -1, next_control_ = 0, next_print_ = 0, target_ = 1, force_ = 0;
};
