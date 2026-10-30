#pragma once
#include <mujoco/mujoco.h>

namespace pendulum {
// One pulse per experiment; all times use simulation seconds.
struct Disturbance {
  double start = 5;
  double duration = 0.2;
  double torque = 2;
  int automatic = 1;
  bool fired = false;
  double active_until = -1;
  double active_torque = 0;

  void Reset() { fired = false; active_until = -1; active_torque = 0; }
  void Trigger(double time) {
    fired = true;
    active_until = time + duration;
    active_torque = torque;
  }
  void Apply(const mjModel* m, mjData* d) {
    const int joint = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");
    if (joint < 0) return;
    if (automatic && !fired && d->time >= start) Trigger(d->time);
    // qfrc_applied persists across steps: explicitly remove the pulse at its end.
    d->qfrc_applied[m->jnt_dofadr[joint]] =
        d->time < active_until ? active_torque : 0;
  }
};
}  // namespace pendulum
