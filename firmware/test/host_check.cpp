// Host harness: reads "y0 y1 y2" lines on stdin, runs the firmware core
// (predict with u_prev -> update -> gates), prints
// "nis eps nis_mode tilt_mode psi_hat u_cmd" per line. Used by
// tests/test_firmware_core.py to prove the C++ matches the Python sim.
#include <cstdio>
#include "../include/echo_core.hpp"

int main() {
  echo::Kalman kf;
  echo::NisGate gate;
  echo::TiltGate tilt;
  double u_prev = 0.0, y[3];
  while (std::scanf("%lf %lf %lf", &y[0], &y[1], &y[2]) == 3) {
    kf.predict(u_prev);
    const double nis = kf.update(y);
    const double eps = gate.step(nis);
    tilt.step(kf.x[1]);
    const double u = echo::balance_u(kf.x);
    const double v = echo::clip_motor(u / 2, ECHO_V_BATT);
    u_prev = 2 * v;
    std::printf("%.12g %.12g %d %d %.12g %.12g\n", nis, eps, (int)gate.mode, (int)tilt.mode, kf.x[1], u);
  }
  return 0;
}
