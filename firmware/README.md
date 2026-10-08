# Echo Balancer on Wokwi (ESP32 NIS gate)

What this is: the **gate** (Kalman filter -> NIS -> windowed epsilon -> NORMAL/CAUTIOUS/HALT, plus the tilt-threshold baseline) running as ESP32 firmware at 200 Hz in [Wokwi](https://wokwi.com).
What it is **not**: a balancing demo. Wokwi has no rigid-body physics, so the robot is a *virtual* one inside the firmware (`echo::VirtualRobot`: the nominal **linear** plant driven by the LQR motor command, with process noise = Q). It is valid near upright only. The balance LQR gain, KF matrices, noise levels and gate thresholds are **generated from `sim/params.py`** (`include/echo_params.h`), never hand-copied.

## Run it
1. Build: `cd firmware && pio run` (PlatformIO, `espressif32@6.9.0`).
2. Open the folder in VS Code with the Wokwi extension (`wokwi.toml` + `diagram.json`), or paste `diagram.json` + `src/main.cpp` + `include/*` into a wokwi.com ESP32 project.
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

LEDs: green NORMAL, yellow CAUTIOUS, red HALT (+ buzzer = the HALT flag). CAUTIOUS/HALT only change the indicators here: there is no speed servo in this firmware, so "speed ref x0.4 / 0" has nothing to act on.

## Hardware mapping / caveats
- TB6612FNG and the GY-87's HMC5883L have no Wokwi part; only the MPU6050 half is wired. Motor voltage is printed (`u_V`) instead of driving a motor.
- Encoder CPR = 360 is still a `PLACEHOLDER` (CLAUDE.md 6.2).
- **Not yet run in Wokwi or built for the ESP32** by the author of this change (the sandbox blocks the PlatformIO registry and Wokwi). `src/main.cpp` was only syntax-checked against Arduino stubs; `diagram.json` pin names (`D21`, `GND.1`, ...) follow the `wokwi-esp32-devkit-v1` convention but are unverified. The filter/gate core *is* verified (below).

## Verification (host, `tests/test_firmware_core.py`)
- The C++ core reproduces `sim/estimator.py` + `sim/gate.py` (NIS, epsilon, both gate modes, psi_hat, u) on identical measurements: rtol 1e-6, modes identical.
- Nominal 60 s x 3 seeds: no mode change; mean NIS ~2.93 (chi2(3) mean = 3); max epsilon ~870 vs tau1 = 1300.
- At onset t = 5 s (seeds 1-3): gyro bias step -> HALT within 0.3-0.5 s; accel noise x5 -> within 0.2 s; accel tilt 0.1 rad -> within 0.2 s.
- **Not detected / honest limits:** a 0.1 rad/s push triggers the gate on 1 of 3 seeds (small, recovers fast; same finding as the Python sim). A payload shift makes the true linear plant unstable under the nominal LQR: the gate does flag it (HALT within 0.5 s) but the virtual robot falls anyway in 2 of 3 seeds, since HALT here cannot change the control law.
