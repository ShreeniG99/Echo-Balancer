# Echo Balancer on Wokwi (ESP32 NIS gate)

What this is: the **gate** (Kalman filter -> NIS -> windowed epsilon -> NORMAL/CAUTIOUS/HALT, plus the tilt-threshold baseline) running as ESP32 firmware at 200 Hz in [Wokwi](https://wokwi.com).
What it is **not**: a balancing demo. Wokwi has no rigid-body physics, so the robot is a *virtual* one inside the firmware (`echo::VirtualRobot`: the nominal **linear** plant driven by the LQR motor command, with process noise = Q). It is valid near upright only. The balance LQR gain, KF matrices, noise levels and gate thresholds are **generated from `sim/params.py`** (`include/echo_params.h`), never hand-copied.

## Run it
1. Build: `cd firmware && pio run` (PlatformIO, `espressif32@6.9.0`, board `esp32-s3-devkitc-1`).
2. Open the folder in VS Code with the Wokwi extension (`wokwi.toml` + `diagram.json`), or paste `diagram.json` + `src/main.cpp` + `include/*` into a wokwi.com **ESP32-S3** project.
3. Open the serial monitor (115200). CSV columns: `t_s,nis,eps,nis_mode,tilt_mode,psi_deg,psi_hat_deg,u_V,fallen`.

Regenerate constants after any change to `sim/params.py`: `PYTHONPATH=. uv run python firmware/tools/gen_params_header.py` (a test fails if the header is stale).

## Injecting disturbances (CLAUDE.md section 8)
| Fault | How |
|---|---|
| Sensor fault (accel tilt / gyro bias) | MPU6050 sliders: the deviation from the boot-time rest pose is added to `psi_acc` / gyro |
| Gyro bias step (0.05 rad/s) | serial `g` |
| Accel noise x5 | serial `a` |
| Push (psi_dot kick 0.1 rad/s) | serial `p` |
| Payload shift (true plant M+1.0 kg, L+0.1 m; KF stays nominal) | serial `m` |
| Wheel slip (encoder offset) | KY-040 knob |
| Clear | serial `r` |

LEDs: green NORMAL, yellow CAUTIOUS, red HALT (+ buzzer = the HALT flag). The mode also drives the control (CLAUDE.md section 10): the speed-servo LQR tracks a wheel-speed reference of 0.3 rad/s in NORMAL, x0.4 with the softer-Q gain set in CAUTIOUS, and 0 in HALT (keeps balancing in place, also on the softer gains). On leaving NORMAL the KF process noise Q is also inflated x100 (`FallbackParams` in `sim/params.py`). Same gains and reference as `sim/run.py`, generated from `sim/params.py`.

Step-by-step first-run instructions: [WOKWI_GUIDE.md](WOKWI_GUIDE.md). Gate comparison plot: `docs/firmware_gate_comparison.png` (regenerate with `PYTHONPATH=. uv run python analysis/plot_firmware_gate.py`).

## Hardware mapping / caveats
- **ESP32-S3 pin map** (GPIO 26-32 are flash/PSRAM and 25 does not exist on the S3): I2C SDA 8 / SCL 9, LEDs 4/5/6 (green/yellow/red), buzzer 7, KY-040 CLK 15 / DT 16.
- TB6612FNG and the GY-87's HMC5883L have no Wokwi part; only the MPU6050 half is wired. Motor voltage is printed (`u_V`) instead of driving a motor.
- Encoder CPR = 360 is still a `PLACEHOLDER` (CLAUDE.md 6.2).
- **Not yet run in Wokwi or built for the ESP32** by the author of this change (the sandbox blocks the PlatformIO registry and Wokwi). `src/main.cpp` was only syntax-checked against Arduino stubs; `diagram.json` pin names (`D21`, `GND.1`, ...) follow the `wokwi-esp32-devkit-v1` convention but are unverified. The filter/gate core *is* verified (below).

## Verification (host, `tests/test_firmware_core.py`)
- The C++ core reproduces `sim/estimator.py` + `sim/gate.py` (NIS, epsilon, both gate modes, psi_hat, u) on identical measurements: rtol 1e-6, modes identical.
- Nominal 60 s x 3 seeds (speed servo on): no mode change; mean NIS ~3.0 (chi2(3) mean = 3); max epsilon <= 701 vs tau1 = 1300.
- At onset t = 5 s (seeds 1-3): gyro bias step -> detected 0.46-0.49 s; accel noise x5 -> 0.11-0.17 s; accel tilt 0.1 rad -> 0.11-0.14 s. In every case HALT zeroes the speed reference and the robot keeps balancing (does not fall).
- The tilt-threshold baseline misses accel noise entirely and reacts late to gyro bias (see the plot): this is the NIS gate's advantage.
- Payload shift (10 seeds): detected 10/10 within 0.3-0.4 s, **before tilt reaches 1 deg**, and the robot is **not** lost (worst max |psi| 7.0 deg, 0/10 falls). Nominal 10 seeds: 0 false alarms.
- **Why payload needed a fallback (corrects an earlier wrong claim that "the LQR can't stabilise it"):** with true-state feedback the heavier plant is stable under the nominal gains (max tilt 0.19 deg). What goes unstable is the *estimate-based* loop, because the KF keeps the nominal model by design (CLAUDE.md section 8). Also, the nominal speed-servo gain is marginal in the sampled 200 Hz loop on that plant (spectral radius 1.0082) while the softer set is stable (0.9993). So on leaving NORMAL the firmware uses the softer gains in both CAUTIOUS and HALT and inflates Q x100 so the KF trusts the sensors over its wrong model. Q x10 and x100 recover; x1000+ overshoot and fall.
- **Not detected / honest limits:** the 0.1 rad/s push is **not** detected on any of 10 seeds (epsilon peaks ~1150 < tau1 = 1300 for ~1 s); the robot recovers by itself. With Q inflation the gyro-bias step now lands in CAUTIOUS and returns to NORMAL once the KF has absorbed the bias, instead of staying in HALT.
- **Python parity:** `sim/run.py` still uses the nominal gain in HALT and no Q inflation (step-6 "Design decision 6"). The fallback is firmware-only for now; the batch metrics in Python are unchanged.

## Short-window spike detector
Since 2026-10-09 the default gate includes a 10-sample (50 ms) spike detector (`ECHO_SHORT_N`, `ECHO_SHORT_TAU`, generated from `sim.params.default_gate_params`). It enters CAUTIOUS only. In Wokwi the `p` (push) key should now turn the LED yellow. `tests/test_firmware_core.py` checks it matches Python mode-for-mode.
