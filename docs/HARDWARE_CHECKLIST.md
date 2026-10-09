# Hardware checklist: what to measure and send back

Every result so far uses the NXTway-GS **simulation v0** numbers. To report results for *our* robot, each parameter below needs a real value. Each value goes into `sim/params.py::hardware_v1_param_set()` with a tag: `MEASURED` (you measured it), `REAL_SPEC` (from a datasheet), or `ESTIMATED` (a calculated guess). Until a value is `MEASURED` or `REAL_SPEC`, results that depend on it can't be reported as real (CLAUDE.md 6.2).

`unreportable_params(param_set("hardware_v1"))` lists what is still missing.

## Item 8 — robot parameters

### A. Tell me which parts you actually bought (5 minutes, no tools)

The motor links decide everything in section C.

| What | Example of what I need |
|---|---|
| Gear motor model and the shop / datasheet link | "JGA25-370, 12 V, 280 rpm, encoder 11 PPR, gear ratio 1:34" |
| Wheel diameter | "65 mm" |
| Encoder type | Hall or optical; pulses per motor revolution |
| Battery | already known: 2S 1500 mAh LiPo (7.4 V nominal, 8.4 V full) |
| IMU | already known: GY-87 (MPU6050 + HMC5883L) |
| Motor driver | already known: TB6612FNG |
| Ultrasonic | HC-SR04? How many, and where do they point? |

### B. Measure with a kitchen scale and a ruler (30 minutes)

| Parameter | Symbol | How |
|---|---|---|
| Mass of each wheel (with tyre, without motor) | m | weigh one wheel |
| Wheel radius | R | measure the diameter and halve it |
| Mass of everything except the two wheels | M | weigh the whole robot, then subtract 2 × wheel |
| Wheel track (distance between the wheel centres) | W | ruler across the axle |
| Body depth and height | D, H | ruler |
| Axle to centre of mass | L | balance the body (wheels off) on a pencil edge laid across it, and measure from the axle line to the balance point. Or hang it from two points and find where the plumb lines cross. |

### C. Motor constants (from the datasheet, or measure: about 1 hour, needs a multimeter)

| Parameter | Symbol | How |
|---|---|---|
| Armature resistance | R_m | multimeter across the motor terminals, wheel held still; average a few shaft positions |
| Back-EMF constant | K_b | spin the motor at a known speed (encoder) unpowered or driven by another motor, and measure the open-circuit voltage. K_b = V / ω (V·s/rad). |
| Torque constant | K_t | in SI units, K_t = K_b (N·m/A). Write down both if the datasheet lists both. |
| Gear ratio | n | datasheet; or count encoder pulses per wheel revolution ÷ pulses per motor revolution |
| Encoder counts per wheel revolution | CPR | turn the wheel exactly 10 times by hand, read the count, divide by 10 |
| Rotor inertia | J_m | datasheet. If absent, I estimate it (tagged `ESTIMATED`). |
| Friction | f_m, f_w | I'll fit these from a coast-down log (section D). No measurement needed from you. |

### D. Two short sensor logs (about 20 minutes, using a small sketch I'll provide)

1. **IMU noise:** robot lying still on the table for 60 s, logging gyro and accelerometer at 200 Hz. This gives σ_gyro, σ_accel and the gyro bias drift.
2. **Coast-down:** robot held upright on a stand with the wheels free. Spin the wheels with a fixed voltage, cut the power, and log the encoder until they stop. This gives the friction terms.
3. **Ultrasonic:** pointed at a wall 0.5 m away for 30 s. This gives the noise σ and the real dropout rate.

Send me the numbers in A–C and the three CSV logs from D. I'll then fill in `hardware_v1`, re-run every experiment on it, re-calibrate the thresholds, and write up what changed.

## Item 9 — firmware for the real robot

### Decisions I need from you
1. **Pin map on your actual board.** Which ESP32-S3 GPIOs go to the TB6612FNG pins (PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY), to the two encoders (A and B for each), to the HC-SR04 sensors (TRIG / ECHO), and to I2C (SDA / SCL)? A photo of the wiring plus a list is enough.
2. **Motor direction.** Which way is "forward" for each motor? We'll find out with one 2-second test.
3. **Safety.** Do you have a physical kill switch, or a way to catch the robot while testing? First tests should run tethered or held, with the motors limited to a low voltage.

### What I'll build once I have those
- A TB6612FNG driver: PWM plus direction pins, with the per-motor clip to ±V_batt.
- Encoder reading on the ESP32-S3 hardware pulse counter (PCNT), converted to the same θ_enc = θ − ψ.
- An HC-SR04 driver with the same dropout handling as the simulation (`hold_on_dropout`).
- The real balance loop: the LQR on the Kalman estimate, replacing the virtual robot.
- A battery-voltage reading through a resistor divider, if you add one. That feeds V_batt and battery-droop detection.
- A bring-up sequence: sensors only, then motors on a stand, then balancing, each step with a pass/fail check you can run.

### Hardware I'd suggest adding (cheap, optional)
- A resistor divider from the battery to an ADC pin, so the robot knows its battery voltage.
- A push-button for the mode or kill switch.
