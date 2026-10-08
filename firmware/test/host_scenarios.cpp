// Host harness: runs the Sim for a scenario and prints per-tick
// "t nis eps nis_mode tilt_mode psi_true psi_hat fallen".
// usage: host_scenarios <scenario> <seed> <seconds> <onset_s>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../include/echo_core.hpp"

int main(int argc, char **argv) {
  if (argc < 5) return 2;
  const char *sc = argv[1];
  echo::Sim sim(std::strtoull(argv[2], nullptr, 10));
  const double secs = std::atof(argv[3]), onset = std::atof(argv[4]);
  const int n = (int)(secs / ECHO_DT);
  bool fired = false;
  for (int k = 0; k < n; ++k) {
    const double t = k * ECHO_DT;
    if (!fired && t >= onset) {
      fired = true;
      if (!std::strcmp(sc, "push")) sim.robot.push();
      else if (!std::strcmp(sc, "gyro_bias")) sim.faults.gyro_bias = ECHO_GYRO_BIAS_STEP;
      else if (!std::strcmp(sc, "accel_noise")) sim.faults.accel_noise_mult = ECHO_ACCEL_NOISE_MULT;
      else if (!std::strcmp(sc, "payload")) sim.faults.payload_shifted = true;
      else if (!std::strcmp(sc, "accel_tilt")) sim.faults.accel_offset = 0.1;
    }
    sim.tick();
    std::printf("%.3f %.9g %.9g %d %d %.9g %.9g %d\n", t, sim.nis, sim.epsilon, (int)sim.nis_gate.mode,
                (int)sim.tilt_gate.mode, sim.robot.x[1], sim.kf.x[1], (int)sim.fallen);
  }
  return 0;
}
