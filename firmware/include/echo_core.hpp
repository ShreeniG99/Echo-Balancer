// Echo Balancer firmware core: Kalman filter + NIS + NIS gate + tilt gate.
// Header-only and Arduino-free so the exact same code is compiled for the
// ESP32 (Wokwi) and for the host cross-check against sim/estimator.py and
// sim/gate.py (tests/test_firmware_core.py). Mirrors the Python line for line.
#pragma once

#include <cmath>
#include <cstdint>
#include "echo_params.h"

namespace echo {

enum class Mode : int { NORMAL = 0, CAUTIOUS = 1, HALT = 2 };

inline const char *mode_name(Mode m) {
  return m == Mode::NORMAL ? "NORMAL" : (m == Mode::CAUTIOUS ? "CAUTIOUS" : "HALT");
}

constexpr int NX = 5;  // [theta, psi, theta_dot, psi_dot, b_g]
constexpr int NY = 3;  // [theta_enc, psi_dot_gyro, psi_acc]

struct Kalman {
  double x[NX] = {0, 0, 0, 0, 0};
  double P[NX * NX];
  double q_scale = 1.0;  // process-noise inflation (fallback: trust sensors over the model)

  Kalman() {
    for (int i = 0; i < NX * NX; ++i) P[i] = ECHO_P0[i];
  }

  // x <- Ad x + Bd u ; P <- Ad P Ad^T + Q (symmetrized)
  void predict(double u) {
    double xn[NX], AP[NX * NX], Pn[NX * NX];
    for (int i = 0; i < NX; ++i) {
      double s = ECHO_BD[i] * u;
      for (int j = 0; j < NX; ++j) s += ECHO_AD[i * NX + j] * x[j];
      xn[i] = s;
    }
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NX; ++j) {
        double s = 0;
        for (int k = 0; k < NX; ++k) s += ECHO_AD[i * NX + k] * P[k * NX + j];
        AP[i * NX + j] = s;
      }
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NX; ++j) {
        double s = q_scale * ECHO_Q[i * NX + j];
        for (int k = 0; k < NX; ++k) s += AP[i * NX + k] * ECHO_AD[j * NX + k];
        Pn[i * NX + j] = s;
      }
    for (int i = 0; i < NX; ++i) {
      x[i] = xn[i];
      for (int j = 0; j < NX; ++j) P[i * NX + j] = 0.5 * (Pn[i * NX + j] + Pn[j * NX + i]);
    }
  }

  // Measurement update (Joseph form). Returns NIS = nu^T S^-1 nu.
  double update(const double y[NY]) {
    double nu[NY], PCt[NX * NY], S[NY * NY];
    for (int i = 0; i < NY; ++i) {
      double s = y[i];
      for (int j = 0; j < NX; ++j) s -= ECHO_C[i * NX + j] * x[j];
      nu[i] = s;
    }
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NY; ++j) {
        double s = 0;
        for (int k = 0; k < NX; ++k) s += P[i * NX + k] * ECHO_C[j * NX + k];
        PCt[i * NY + j] = s;
      }
    for (int i = 0; i < NY; ++i)
      for (int j = 0; j < NY; ++j) {
        double s = ECHO_R[i * NY + j];
        for (int k = 0; k < NX; ++k) s += ECHO_C[i * NX + k] * PCt[k * NY + j];
        S[i * NY + j] = s;
      }
    double Sinv[NY * NY];
    invert3(S, Sinv);

    double nis = 0;
    for (int i = 0; i < NY; ++i)
      for (int j = 0; j < NY; ++j) nis += nu[i] * Sinv[i * NY + j] * nu[j];

    double K[NX * NY];
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NY; ++j) {
        double s = 0;
        for (int k = 0; k < NY; ++k) s += PCt[i * NY + k] * Sinv[k * NY + j];
        K[i * NY + j] = s;
      }
    for (int i = 0; i < NX; ++i) {
      double s = 0;
      for (int j = 0; j < NY; ++j) s += K[i * NY + j] * nu[j];
      x[i] += s;
    }
    // P <- (I-KC) P (I-KC)^T + K R K^T
    double IKC[NX * NX], T[NX * NX], Pn[NX * NX];
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NX; ++j) {
        double s = (i == j) ? 1.0 : 0.0;
        for (int k = 0; k < NY; ++k) s -= K[i * NY + k] * ECHO_C[k * NX + j];
        IKC[i * NX + j] = s;
      }
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NX; ++j) {
        double s = 0;
        for (int k = 0; k < NX; ++k) s += IKC[i * NX + k] * P[k * NX + j];
        T[i * NX + j] = s;
      }
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NX; ++j) {
        double s = 0;
        for (int k = 0; k < NX; ++k) s += T[i * NX + k] * IKC[j * NX + k];
        for (int a = 0; a < NY; ++a)
          for (int b = 0; b < NY; ++b)
            s += K[i * NY + a] * ECHO_R[a * NY + b] * K[j * NY + b];
        Pn[i * NX + j] = s;
      }
    for (int i = 0; i < NX * NX; ++i) P[i] = Pn[i];
    return nis;
  }

 private:
  static void invert3(const double m[9], double out[9]) {
    const double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5],
                 g = m[6], h = m[7], i = m[8];
    const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    const double id = 1.0 / det;
    out[0] = (e * i - f * h) * id; out[1] = (c * h - b * i) * id; out[2] = (b * f - c * e) * id;
    out[3] = (f * g - d * i) * id; out[4] = (a * i - c * g) * id; out[5] = (c * d - a * f) * id;
    out[6] = (d * h - e * g) * id; out[7] = (b * g - a * h) * id; out[8] = (a * e - b * d) * id;
  }
};

// Mirrors sim/gate.py::step_gate (windowed NIS statistic, hysteresis, dwell).
struct NisGate {
  Mode mode = Mode::NORMAL;
  double window[ECHO_GATE_N];
  int count = 0;   // samples accumulated (saturates at N)
  int head = 0;    // next write slot
  double time_in_mode = 0.0;
  double epsilon = 0.0;

  // Returns epsilon_k.
  double step(double nis) {
    window[head] = nis;
    head = (head + 1) % ECHO_GATE_N;
    if (count < ECHO_GATE_N) ++count;
    const bool full = (count == ECHO_GATE_N);
    double eps = 0;
    for (int i = 0; i < count; ++i) eps += window[i];
    epsilon = eps;

    time_in_mode += ECHO_DT;
    const bool can = time_in_mode >= ECHO_T_DWELL;
    Mode nm = mode;
    if (can && full) {
      if (mode == Mode::NORMAL) {
        if (eps > ECHO_TAU2) nm = Mode::HALT;
        else if (eps > ECHO_TAU1) nm = Mode::CAUTIOUS;
      } else if (mode == Mode::CAUTIOUS) {
        if (eps > ECHO_TAU2) nm = Mode::HALT;
        else if (eps < ECHO_TAU1_EXIT) nm = Mode::NORMAL;
      } else {
        if (eps < ECHO_TAU2_EXIT) nm = Mode::CAUTIOUS;
      }
    }
    if (nm != mode) time_in_mode = 0.0;
    mode = nm;
    return eps;
  }
};

// Mirrors sim/gate.py::step_tilt_gate (baseline: |psi_hat| thresholds only).
struct TiltGate {
  Mode mode = Mode::NORMAL;
  double time_in_mode = 0.0;

  void step(double psi_hat) {
    const double a = std::fabs(psi_hat);
    time_in_mode += ECHO_DT;
    const bool can = time_in_mode >= ECHO_TILT_T_DWELL;
    Mode nm = mode;
    if (can) {
      if (mode == Mode::NORMAL) {
        if (a > ECHO_TILT_PSI2) nm = Mode::HALT;
        else if (a > ECHO_TILT_PSI1) nm = Mode::CAUTIOUS;
      } else if (mode == Mode::CAUTIOUS) {
        if (a > ECHO_TILT_PSI2) nm = Mode::HALT;
        else if (a < ECHO_TILT_PSI1_EXIT) nm = Mode::NORMAL;
      } else {
        if (a < ECHO_TILT_PSI2_EXIT) nm = Mode::CAUTIOUS;
      }
    }
    if (nm != mode) time_in_mode = 0.0;
    mode = nm;
  }
};

// Balance LQR: u = v_l + v_r = -K x_hat[0..3]; per-motor clip to +/-V_batt.
inline double balance_u(const double x[NX]) {
  double u = 0;
  for (int i = 0; i < 4; ++i) u -= ECHO_K_BALANCE[i] * x[i];
  return u;
}
inline double clip_motor(double v, double v_batt) {
  return v > v_batt ? v_batt : (v < -v_batt ? -v_batt : v);
}


// ---------------------------------------------------------------------------
// Virtual robot ("software-in-the-loop" plant). Wokwi has no rigid-body
// physics, so the true robot is the nominal linear model, driven by the motor
// command, plus process noise matching Q. Disturbances (CLAUDE.md section 8)
// are injected here; the estimator keeps the NOMINAL model, so the innovation
// sees the mismatch. Linear model: valid near upright only (|psi| small).
// ---------------------------------------------------------------------------
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 1) : s(seed ? seed : 1) {}
  double uniform() {  // xorshift64*, in (0,1)
    s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
    return ((s * 0x2545F4914F6CDD1DULL) >> 11) * (1.0 / 9007199254740992.0) + 1e-18;
  }
  double gauss() { return std::sqrt(-2.0 * std::log(uniform())) * std::cos(6.283185307179586 * uniform()); }
};

struct Faults {
  double gyro_bias = 0.0;        // rad/s added to gyro (sensor fault, or MPU slider offset)
  double accel_offset = 0.0;     // rad added to psi_acc (MPU accel slider offset)
  double accel_noise_mult = 1.0; // x sigma_accel (sensor fault)
  double enc_offset = 0.0;       // rad added to theta_enc (wheel-slip knob)
  bool payload_shifted = false;  // true plant switches to the payload-shifted model
};

struct VirtualRobot {
  double x[NX] = {0, 0, 0, 0, 0};  // true [theta, psi, theta_dot, psi_dot, b_g]
  Rng rng;
  explicit VirtualRobot(uint64_t seed) : rng(seed) {}

  void push() { x[3] += ECHO_PUSH_KICK; }

  // Advance one control interval under zero-order-hold u = v_l + v_r.
  void step(double u, bool payload) {
    const double *A = payload ? ECHO_AD_PAYLOAD : ECHO_AD;
    const double *B = payload ? ECHO_BD_PAYLOAD : ECHO_BD;
    double xn[NX];
    for (int i = 0; i < NX; ++i) {
      double s = B[i] * u;
      for (int j = 0; j < NX; ++j) s += A[i * NX + j] * x[j];
      xn[i] = s + std::sqrt(ECHO_Q[i * NX + i]) * rng.gauss();
    }
    for (int i = 0; i < NX; ++i) x[i] = xn[i];
  }

  void measure(const Faults &f, double y[NY]) {
    const double step = 6.283185307179586 / ECHO_ENCODER_CPR;
    const double enc = x[0] - x[1] + f.enc_offset;
    y[0] = std::round(enc / step) * step;
    y[1] = x[3] + x[4] + f.gyro_bias + ECHO_SIGMA_GYRO * rng.gauss();
    y[2] = x[1] + f.accel_offset + ECHO_SIGMA_ACCEL * f.accel_noise_mult * rng.gauss();
  }
};

// One 200 Hz tick of the whole firmware pipeline.
struct Sim {
  VirtualRobot robot;
  Kalman kf;
  NisGate nis_gate;
  TiltGate tilt_gate;
  Faults faults;
  double u_prev = 0.0;
  double theta_ref = 0.0, z_int = 0.0, speed_ref = 0.0;  // speed servo state (sim/run.py)
  double nis = 0.0, epsilon = 0.0, u_cmd = 0.0;
  bool fallen = false;

  explicit Sim(uint64_t seed) : robot(seed) {}

  void tick() {
    if (!fallen) robot.step(u_prev, faults.payload_shifted);  // a fallen robot lies still
    double y[NY];
    robot.measure(faults, y);
    kf.predict(u_prev);
    nis = kf.update(y);
    epsilon = nis_gate.step(nis);
    tilt_gate.step(kf.x[1]);
    if (std::fabs(robot.x[1]) > ECHO_FALL_PSI) fallen = true;  // motors off on fall only
    // Gate -> control (CLAUDE.md section 10): NORMAL full speed, CAUTIOUS x0.4 with the
    // softer-Q gain set, HALT speed 0 (keeps balancing in place, softer gains). Uses the estimate x_hat.
    const Mode m = nis_gate.mode;
    kf.q_scale = (m == Mode::NORMAL) ? 1.0 : ECHO_FALLBACK_Q_SCALE;  // see FallbackParams
    // CAUTIOUS and HALT both use the softer-Q gain set: it is stable in the sampled 200 Hz loop on the
    // nominal AND payload-shifted plants (spectral radius 0.9987 / 0.9993), the nominal set is not (1.0082).
    const double *K5 = (m == Mode::NORMAL) ? ECHO_K5_NOMINAL : ECHO_K5_CAUTIOUS;
    speed_ref = (m == Mode::NORMAL) ? ECHO_SPEED_REF_NOMINAL
              : (m == Mode::CAUTIOUS ? ECHO_SPEED_REF_NOMINAL * ECHO_CAUTIOUS_SPEED_SCALE : 0.0);
    const double e[5] = {kf.x[0] - theta_ref, kf.x[1], kf.x[2] - speed_ref, kf.x[3], z_int};
    theta_ref += speed_ref * ECHO_DT;
    u_cmd = 0;
    for (int i = 0; i < 5; ++i) u_cmd -= K5[i] * e[i];
    z_int += (kf.x[0] - theta_ref) * ECHO_DT;
    const double v = fallen ? 0.0 : clip_motor(u_cmd / 2, ECHO_V_BATT);
    u_prev = 2 * v;
  }
};

}  // namespace echo
